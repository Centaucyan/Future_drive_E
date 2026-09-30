let currentRobotId = 1;

// 로봇별 시뮬레이션 상태 데이터
const robotStates = {
    1: { name: 'Robot 1', battery: 92, mileage: 14.8, speed: 1.2 },
    2: { name: 'Robot 2', battery: 67, mileage: 8.3, speed: 0.8 }
};

function switchRobot(id) {
    currentRobotId = id;
    const btn1 = document.getElementById('btn-r1');
    const btn2 = document.getElementById('btn-r2');
    const title = document.getElementById('currentRobotTitle');

    if (id === 1) {
        btn1.className = "flex items-center gap-3 px-6 py-2 rounded-full text-sm font-medium transition-all bg-blue-600 text-white shadow-md border border-blue-500";
        btn2.className = "flex items-center gap-3 px-6 py-2 rounded-full text-sm font-medium text-slate-400 hover:text-white transition-all bg-slate-900/60";
        if (title) title.innerText = "Robot 1 Control";
    } else {
        btn2.className = "flex items-center gap-3 px-6 py-2 rounded-full text-sm font-medium transition-all bg-amber-600 text-white shadow-md border border-amber-500";
        btn1.className = "flex items-center gap-3 px-6 py-2 rounded-full text-sm font-medium text-slate-400 hover:text-white transition-all bg-slate-900/60";
        if (title) title.innerText = "Robot 2 Control";
    }

    // 맵 선택 로봇 변경
    if (window.mapRenderer) {
        window.mapRenderer.setActiveRobot(id);
    }

    // 센서 데이터 표기 업데이트
    updateDashboardUI();
}

function updateDashboardUI() {
    const data = robotStates[currentRobotId];
    if (!data) return;

    const batElem = document.getElementById('robotBattery');
    const distElem = document.getElementById('robotMileage');
    const speedElem = document.getElementById('robotSpeed');

    if (batElem) batElem.innerText = `${data.battery}%`;
    if (distElem) distElem.innerText = `${data.mileage} km`;
    if (speedElem) speedElem.innerText = `${data.speed} m/s`;
}

function applyRoute() {
    const orig = document.getElementById('originInput').value;
    const dest = document.getElementById('destInput').value;

    if (window.mapRenderer) {
        const startX = Math.random() * 300 + 100;
        const startY = Math.random() * 300 + 100;
        const newWaypoints = [
            { x: startX, y: startY },
            { x: startX + 180, y: startY + 80 },
            { x: startX + 320, y: startY + 160 }
        ];
        window.mapRenderer.setRoute(currentRobotId, newWaypoints);
        alert(`[Robot ${currentRobotId}] 경로 설정 완료\n출발: ${orig || '현재위치'} ➔ 도착: ${dest || '목적지'}`);
    }
}

async function triggerMasterEmergencyStop() {
    try {
        const response = await fetch('/api/master-emergency-stop', { method: 'POST' });
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        const banner = document.getElementById('masterEStopBanner');
        if (banner) banner.classList.remove('hidden');
    } catch (error) {
        alert(`긴급정지 명령을 확인하지 못했습니다. EP 연동 상태를 확인하세요. (${error.message})`);
    }
}

function resetMasterEmergencyStop() {
    alert('정지 해제는 EP 연동 후 사용할 수 있습니다.');
}

// 실시간 주행 거리 및 배터리 소모 시뮬레이션 루프
setInterval(() => {
    Object.keys(robotStates).forEach(id => {
        robotStates[id].mileage = (parseFloat(robotStates[id].mileage) + 0.001).toFixed(2);
    });
    updateDashboardUI();
}, 1000);

document.addEventListener('DOMContentLoaded', () => {
    switchRobot(1);
});
