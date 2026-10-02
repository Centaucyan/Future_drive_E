"""실행 진입점: python3 -m app.app"""
import os

import uvicorn


if __name__ == "__main__":
    uvicorn.run(
        "app.main:app",
        host=os.getenv("RVIZ_WEB_HOST", "127.0.0.1"),
        port=int(os.getenv("RVIZ_WEB_PORT", "8081")),
        reload=False,
    )
