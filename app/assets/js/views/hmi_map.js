class HMIMapRenderer {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        if (!this.canvas) return;
        this.ctx = this.canvas.getContext('2d');

        // 로봇 초기 위치 및 경로 데이터
        this.robots = {
            1: {
                id: 1,
                name: 'Robot 1',
                x: 200,
                y: 300,
                color: '#3b82f6', // 파란색
                path: [{x: 200, y: 300}, {x: 400, y: 250}, {x: 600, y: 400}, {x: 800, y: 300}],
                targetIndex: 0
            },
            2: {
                id: 2,
                name: 'Robot 2',
                x: 300,
                y: 500,
                color: '#f59e0b', // 주황색
                path: [{x: 300, y: 500}, {x: 500, y: 450}, {x: 700, y: 200}],
                targetIndex: 0
            }
        };

        this.activeRobotId = 1;
        this.initCanvasSize();
        window.addEventListener('resize', () => this.initCanvasSize());
        this.startAnimationLoop();
    }

    initCanvasSize() {
        if (!this.canvas) return;
        this.canvas.width = window.innerWidth;
        this.canvas.height = window.innerHeight;
    }

    setActiveRobot(id) {
        this.activeRobotId = id;
    }

    setRoute(robotId, waypoints) {
        if (this.robots[robotId]) {
            this.robots[robotId].path = waypoints;
            this.robots[robotId].x = waypoints[0].x;
            this.robots[robotId].y = waypoints[0].y;
            this.robots[robotId].targetIndex = 0;
        }
    }

    // 🎨 캔버스에 맵 그리드, 경로 라인, 로봇 그리기
    draw() {
        if (!this.ctx) return;
        const width = this.canvas.width;
        const height = this.canvas.height;

        // 1. 배경 청소
        this.ctx.fillStyle = '#0f172a'; // dark slate
        this.ctx.fillRect(0, 0, width, height);

        // 2. 그리드 모눈종이 배경 그리기
        this.ctx.strokeStyle = '#1e293b';
        this.ctx.lineWidth = 1;
        const gridSize = 40;
        for (let x = 0; x < width; x += gridSize) {
            this.ctx.beginPath();
            this.ctx.moveTo(x, 0);
            this.ctx.lineTo(x, height);
            this.ctx.stroke();
        }
        for (let y = 0; y < height; y += gridSize) {
            this.ctx.beginPath();
            this.ctx.moveTo(0, y);
            this.ctx.lineTo(width, y);
            this.ctx.stroke();
        }

        // 3. 로봇별 이동 경로(Path) 및 목적지 점 그리기
        Object.values(this.robots).forEach(bot => {
            if (bot.path && bot.path.length > 1) {
                // 경로 점선/실선 그리기
                this.ctx.beginPath();
                this.ctx.strokeStyle = bot.id === this.activeRobotId ? bot.color : '#475569';
                this.ctx.lineWidth = bot.id === this.activeRobotId ? 3 : 1.5;
                this.ctx.setLineDash(bot.id === this.activeRobotId ? [8, 6] : [4, 4]);

                this.ctx.moveTo(bot.path[0].x, bot.path[0].y);
                for (let i = 1; i < bot.path.length; i++) {
                    this.ctx.lineTo(bot.path[i].x, bot.path[i].y);
                }
                this.ctx.stroke();
                this.ctx.setLineDash([]); // 점선 해제

                // 경로 상의 웨이포인트 노드 그리기
                bot.path.forEach((pt, idx) => {
                    this.ctx.beginPath();
                    this.ctx.arc(pt.x, pt.y, 5, 0, Math.PI * 2);
                    this.ctx.fillStyle = idx === bot.path.length - 1 ? '#ef4444' : bot.color;
                    this.ctx.fill();
                });
            }
        });

        // 4. 로봇 현재 위치 아이콘 그리기
        Object.values(this.robots).forEach(bot => {
            // 로봇 외곽 후광(Glow) 효과
            if (bot.id === this.activeRobotId) {
                this.ctx.beginPath();
                this.ctx.arc(bot.x, bot.y, 22, 0, Math.PI * 2);
                this.ctx.fillStyle = bot.color + '33'; // 반투명
                this.ctx.fill();
            }

            // 로봇 본체 원
            this.ctx.beginPath();
            this.ctx.arc(bot.x, bot.y, 12, 0, Math.PI * 2);
            this.ctx.fillStyle = bot.color;
            this.ctx.shadowColor = bot.color;
            this.ctx.shadowBlur = 10;
            this.ctx.fill();
            this.ctx.shadowBlur = 0; // 그림자 초기화

            // 로봇 텍스트 이름 표기
            this.ctx.fillStyle = '#ffffff';
            this.ctx.font = 'bold 11px sans-serif';
            this.ctx.fillText(bot.name, bot.x - 20, bot.y - 18);
        });
    }

    startAnimationLoop() {
        const loop = () => {
            // 로봇을 경로를 따라 조금씩 이동시키는 시뮬레이션
            Object.values(this.robots).forEach(bot => {
                if (bot.path && bot.path.length > 0) {
                    const target = bot.path[bot.targetIndex];
                    if (target) {
                        const dx = target.x - bot.x;
                        const dy = target.y - bot.y;
                        const dist = Math.sqrt(dx * dx + dy * dy);
                        if (dist < 2) {
                            bot.targetIndex = (bot.targetIndex + 1) % bot.path.length;
                        } else {
                            bot.x += (dx / dist) * 0.8;
                            bot.y += (dy / dist) * 0.8;
                        }
                    }
                }
            });

            this.draw();
            requestAnimationFrame(loop);
        };
        loop();
    }
}

// Global instance 생성
window.mapRenderer = null;
document.addEventListener('DOMContentLoaded', () => {
    window.mapRenderer = new HMIMapRenderer('mapCanvas');
});
