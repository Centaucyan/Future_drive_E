"""로봇별 ROS Domain에 연결되는 rosbridge 프로세스 관리자."""
from __future__ import annotations

import asyncio
import ipaddress
import os
import signal
from dataclasses import dataclass
from pathlib import Path


@dataclass
class ManagedBridge:
    robot_id: str
    robot_ip: str | None
    domain_id: int
    port: int
    process: asyncio.subprocess.Process
    profile_path: Path | None
    log_handle: object


class BridgeManager:
    def __init__(self, runtime_dir: Path, first_port: int = 9100) -> None:
        self.runtime_dir = runtime_dir
        self.first_port = first_port
        self._bridges: dict[str, ManagedBridge] = {}
        self._lock = asyncio.Lock()

    @staticmethod
    def validate(robot_id: str, robot_ip: str | None, domain_id: int) -> tuple[str, str | None, int]:
        clean_id = "".join(c for c in robot_id if c.isalnum() or c in "_-")[:32]
        if not clean_id:
            raise ValueError("로봇 식별자가 올바르지 않습니다.")
        address = None
        if robot_ip:
            try:
                address = ipaddress.ip_address(robot_ip)
            except ValueError as error:
                raise ValueError("로봇 IP 주소가 올바르지 않습니다.") from error
            if address.version != 4:
                raise ValueError("현재는 IPv4 로봇 주소만 지원합니다.")
        if not 0 <= domain_id <= 232:
            raise ValueError("ROS Domain ID는 0~232 범위여야 합니다.")
        return clean_id, str(address) if address else None, domain_id

    def _available_port(self, robot_id: str) -> int:
        existing = self._bridges.get(robot_id)
        if existing:
            return existing.port
        used = {bridge.port for bridge in self._bridges.values()}
        port = next(
            (
                candidate
                for candidate in range(self.first_port, self.first_port + 100)
                if candidate not in used
            ),
            None,
        )
        if port is None:
            # StopIteration이 500 오류로 노출되지 않도록 운영자가 이해할 수 있는 오류로 바꾼다.
            raise RuntimeError("사용 가능한 rosbridge 포트가 없습니다.")
        return port

    def _write_profile(self, robot_id: str, robot_ip: str, domain_id: int) -> Path:
        # Fast DDS discovery ports: PB + DG * domain (+ d1 + PG * participant).
        discovery_ports = [7400 + 250 * domain_id]
        discovery_ports.extend(7410 + 250 * domain_id + 2 * participant for participant in range(0, 121))
        locators = "\n".join(
            f"<locator><udpv4><address>{robot_ip}</address><port>{port}</port></udpv4></locator>"
            for port in discovery_ports
        )
        profile = f"""<?xml version=\"1.0\" encoding=\"UTF-8\" ?>
<profiles xmlns=\"http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles\">
  <participant profile_name=\"web_bridge_{robot_id}\" is_default_profile=\"true\">
    <rtps><builtin><initialPeersList>{locators}</initialPeersList></builtin></rtps>
  </participant>
</profiles>
"""
        path = self.runtime_dir / f"fastdds-{robot_id}.xml"
        path.write_text(profile, encoding="utf-8")
        return path

    async def connect(self, robot_id: str, domain_id: int, robot_ip: str | None = None) -> dict:
        robot_id, robot_ip, domain_id = self.validate(robot_id, robot_ip, domain_id)
        async with self._lock:
            current = self._bridges.get(robot_id)
            if current and current.process.returncode is None:
                if current.robot_ip == robot_ip and current.domain_id == domain_id:
                    return self._describe(current)
                await self._stop(current)

            profile_path = None
            log_handle = None
            try:
                # 아래 준비 과정 중 하나라도 실패하면 except에서 이미 연 자원을 모두 정리한다.
                self.runtime_dir.mkdir(parents=True, exist_ok=True)
                port = self._available_port(robot_id)
                profile_path = (
                    self._write_profile(robot_id, robot_ip, domain_id)
                    if robot_ip
                    else None
                )
                log_path = self.runtime_dir / f"rosbridge-{robot_id}.log"
                log_handle = log_path.open("ab", buffering=0)
                environment = os.environ.copy()
                environment.update(
                    ROS_DOMAIN_ID=str(domain_id),
                    ROS_LOCALHOST_ONLY="0",
                    RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
                    ROS_LOG_DIR=str(self.runtime_dir / "ros"),
                )
                if profile_path:
                    environment["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(profile_path)
                    environment["FASTDDS_DEFAULT_PROFILES_FILE"] = str(profile_path)
                else:
                    environment.pop("FASTRTPS_DEFAULT_PROFILES_FILE", None)
                    environment.pop("FASTDDS_DEFAULT_PROFILES_FILE", None)
                command = (
                    "source /opt/ros/humble/setup.bash && exec ros2 launch rosbridge_server "
                    "rosbridge_websocket_launch.xml address:=0.0.0.0 "
                    f"port:={port} send_action_goals_in_new_thread:=true "
                    "call_services_in_new_thread:=true default_call_service_timeout:=5.0"
                )
                process = await asyncio.create_subprocess_exec(
                    "/bin/bash",
                    "-lc",
                    command,
                    env=environment,
                    stdout=log_handle,
                    stderr=asyncio.subprocess.STDOUT,
                    start_new_session=True,
                )
            except (OSError, RuntimeError) as error:
                # 실패한 시작 시도의 파일 핸들과 임시 DDS 프로필이 다음 연결을 방해하지 않게 한다.
                if log_handle and not log_handle.closed:
                    log_handle.close()
                if profile_path:
                    profile_path.unlink(missing_ok=True)
                raise RuntimeError(f"rosbridge 실행 준비에 실패했습니다: {error}") from error
            bridge = ManagedBridge(robot_id, robot_ip, domain_id, port, process, profile_path, log_handle)
            self._bridges[robot_id] = bridge

        await asyncio.sleep(1.0)
        if process.returncode is not None:
            # 실행 직후 죽은 프로세스를 정상 bridge로 남겨두지 않는다.
            async with self._lock:
                if self._bridges.get(robot_id) is bridge:
                    self._bridges.pop(robot_id, None)
            if not log_handle.closed:
                log_handle.close()
            if profile_path:
                profile_path.unlink(missing_ok=True)
            raise RuntimeError(f"rosbridge가 시작되지 않았습니다. 로그: {log_path}")
        return self._describe(bridge)

    @staticmethod
    def _describe(bridge: ManagedBridge) -> dict:
        return {
            "robot_id": bridge.robot_id,
            "robot_ip": bridge.robot_ip,
            "domain_id": bridge.domain_id,
            "port": bridge.port,
            "running": bridge.process.returncode is None,
        }

    async def _stop(self, bridge: ManagedBridge) -> None:
        if bridge.process.returncode is None:
            try:
                os.killpg(bridge.process.pid, signal.SIGTERM)
                await asyncio.wait_for(bridge.process.wait(), timeout=5)
            except ProcessLookupError:
                pass
            except asyncio.TimeoutError:
                # 정상 종료가 5초를 넘으면 강제 종료하되 이미 사라진 프로세스는 오류로 보지 않는다.
                try:
                    os.killpg(bridge.process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                await bridge.process.wait()
        if not bridge.log_handle.closed:
            bridge.log_handle.close()
        if bridge.profile_path:
            bridge.profile_path.unlink(missing_ok=True)

    async def disconnect(self, robot_id: str) -> bool:
        clean_id = self.validate(robot_id, None, 0)[0]
        async with self._lock:
            bridge = self._bridges.get(clean_id)
            if not bridge:
                return False
            await self._stop(bridge)
            if self._bridges.get(clean_id) is bridge:
                self._bridges.pop(clean_id, None)
            return True

    async def close(self) -> None:
        async with self._lock:
            bridges = list(self._bridges.values())
            failures = []
            for bridge in bridges:
                # 한 로봇의 종료 실패 때문에 다른 rosbridge 정리까지 중단되지 않도록 개별 처리한다.
                try:
                    await self._stop(bridge)
                except (OSError, RuntimeError) as error:
                    failures.append(f"{bridge.robot_id}: {error}")
                else:
                    if self._bridges.get(bridge.robot_id) is bridge:
                        self._bridges.pop(bridge.robot_id, None)
            if failures:
                raise RuntimeError(
                    "일부 rosbridge를 종료하지 못했습니다: " + "; ".join(failures)
                )


bridge_manager = BridgeManager(Path(__file__).resolve().parents[1] / "runtime")
