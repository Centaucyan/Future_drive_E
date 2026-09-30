class HMICamViewer {
    constructor() {
        this.detectionData = [];
        this.initYoloFetcher();
    }

    initYoloFetcher() {
        // 0.2초마다 서버에서 YOLO JSON 결과 받아오기
        setInterval(async () => {
            try {
                const res = await fetch('/api/yolo-detections');
                if (!res.ok) throw new Error(`HTTP ${res.status}`);
                const data = await res.json();
                this.detectionData = data.detections || [];
                this.updateYoloUI();
            } catch (e) {}
        }, 200);
    }

    updateYoloUI() {
        const badge = document.getElementById('yoloDetectionBadge');
        if (badge) {
            badge.innerText = `YOLO: ${this.detectionData.length}개 객체 감지`;
        }
    }
}

document.addEventListener('DOMContentLoaded', () => {
    new HMICamViewer();
});
