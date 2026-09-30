"""기존 실행 명령 지원: python3 app/app.py"""
# 직접 실행 시 FastAPI 서버 시작
if __name__ == "__main__":
    # 1. 실행 경로 및 웹 서버 모듈 준비
    import sys
    from pathlib import Path
    import uvicorn

    # 어느 폴더에서 실행해도 app 패키지를 찾도록 프로젝트 경로 등록
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    # 8080 포트에서 웹 요청 수신
    uvicorn.run("app.main:app", host="0.0.0.0", port=8080)
