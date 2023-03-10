import logging
logging.basicConfig(
    level=logging.DEBUG,
    format='[%(asctime)s] [%(levelname)s] [%(filename)s:%(lineno)s]: %(message)s',
    handlers=[
        logging.FileHandler('remote_bot_system.log'),
        logging.StreamHandler()
    ]
)

import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from tortoise.contrib.fastapi import register_tortoise

from api import user, robot, code_server, ftp_info, login


app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


register_tortoise(
    app,
    db_url="mysql://root:root@127.0.0.1:3306/cloud_test_bench",
    modules={"models": ["models.robot", "models.user", "models.ftp"]},
    generate_schemas=True,
    add_exception_handlers=True,
)


app.include_router(robot.router)
app.include_router(user.router)
app.include_router(code_server.router)
app.include_router(ftp_info.router)
app.include_router(login.router)


if __name__ == "__main__":
    uvicorn.run(app, host="192.168.124.134", port=8000)