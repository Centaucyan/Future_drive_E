export function initialPoseMessage(pose, nowMs = Date.now()) {
  if (![pose.x, pose.y, pose.z, pose.yaw].every(Number.isFinite) || !pose.frame) throw Error('초기 위치 좌표를 확인하세요.');
  const covariance = Array(36).fill(0);
  covariance[0] = .25; covariance[7] = .25; covariance[35] = (Math.PI / 12) ** 2;
  const sec = Math.floor(nowMs / 1000), nanosec = Math.floor((nowMs - sec * 1000) * 1e6);
  return {header: {frame_id: pose.frame, stamp: {sec, nanosec}}, pose: {pose: {position: {x: pose.x, y: pose.y, z: pose.z}, orientation: {x: 0, y: 0, z: Math.sin(pose.yaw / 2), w: Math.cos(pose.yaw / 2)}}, covariance}};
}

// Autonomy Studio bridge HTTP control client. State data uses /ws/state separately.
export class Navigation {
  constructor(report, changed) {
    this.report = report;
    this.changed = changed;
    this.controlApi = '';
    this.active = null;
    this.pendingStop = null;
    this.sawRunning = false;
    this.remoteBusy = false;
  }
  get busy() { return Boolean(this.active || this.pendingStop || this.remoteBusy); }
  get ready() { return Boolean(this.controlApi); }
  setRemoteBusy(value) {
    const next = Boolean(value);
    if (this.remoteBusy === next) return;
    this.remoteBusy = next;
    this.changed();
  }
  connectPlatform(url) {
    this.controlApi = (url || '').replace(/\/+$/, '');
    this.changed();
  }
  async post(path, payload) {
    if (!this.ready) throw Error('플랫폼 제어 연결이 없습니다.');
    const response = await fetch(this.controlApi + path, {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: payload === undefined ? undefined : JSON.stringify(payload)
    });
    if (!response.ok) {
      let detail = `HTTP ${response.status}`;
      try { detail = (await response.json()).detail || detail; } catch {}
      throw Error(detail);
    }
    try { return await response.json(); } catch { return {}; }
  }
  setInitialPose(pose) {
    if (this.busy) throw Error('주행을 정지한 뒤 초기 위치를 설정하세요.');
    this.report('초기 위치 전송 중 · 플랫폼 응답을 기다립니다.');
    this.post('/initialpose', {x: pose.x, y: pose.y, yaw: pose.yaw, frame_id: pose.frame})
      .then(() => this.report('초기 위치 전송 완료 · 위치와 TF 갱신을 기다립니다.'))
      .catch(error => this.report(`초기 위치 전송 실패: ${error.message}`))
      .finally(() => this.changed());
  }
  start(goal) {
    if (this.busy) throw Error('진행 중인 주행을 먼저 정지하세요.');
    this.active = crypto.randomUUID();
    this.sawRunning = false;
    this.report('목적지 전송 · Nav2 응답 대기');
    this.changed();
    this.post('/navigate_to_pose', {x: goal.x, y: goal.y, yaw: goal.yaw, frame_id: goal.frame})
      .then(result => {
        if (result.ok === false || result.accepted === false) {
          throw Error(result.message || 'Nav2가 목적지를 거절했습니다.');
        }
        if (result.goal_id) this.active = result.goal_id;
        this.report('자율주행 시작 · 플랫폼 상태 수신 중');
      })
      .catch(error => {
        this.active = null;
        this.report(`목적지 전송 실패: ${error.message}`);
      })
      .finally(() => this.changed());
  }
  stop() {
    if (this.pendingStop) return;
    if (!this.ready) throw Error('연결이 없어 정지 명령을 전달할 수 없습니다.');
    this.pendingStop = crypto.randomUUID();
    this.report('정지 요청 중 · 주행 취소 및 속도 0 전송');
    this.changed();
    Promise.allSettled([this.post('/cancel_navigation'), this.post('/stop')]).then(results => {
      this.pendingStop = null;
      this.active = null;
      const failed = results.filter(result => result.status === 'rejected');
      this.report(failed.length === results.length ? `정지 실패: ${failed[0].reason.message}` : '정지 명령 전송 완료 · 실제 로봇 상태를 확인하세요.');
      this.changed();
    });
  }
  sync(status) {
    const value = String(status?.status || '').toLowerCase();
    if (['active', 'accepted', 'executing', 'running', 'navigating'].includes(value)) {
      const changed = !this.remoteBusy;
      this.sawRunning = true;
      this.remoteBusy = true;
      if (changed) {
        this.report('자율주행 중');
        this.changed();
      }
    } else if (['idle', 'succeeded', 'completed', 'success'].includes(value)) {
      const wasBusy = this.busy;
      this.active = null;
      this.sawRunning = false;
      this.remoteBusy = false;
      if (wasBusy) this.report('목적지 도착 또는 주행 종료');
      if (wasBusy) this.changed();
    } else if (['failed', 'aborted', 'canceled', 'cancelled'].includes(value)) {
      this.active = null;
      this.sawRunning = false;
      this.remoteBusy = false;
      this.report(`주행 종료: ${value}`);
      this.changed();
    }
  }
  close() {
    this.controlApi = '';
    this.active = null;
    this.pendingStop = null;
    this.sawRunning = false;
    this.remoteBusy = false;
  }
}
