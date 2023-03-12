import logging
from fastapi import FastAPI
from tortoise.contrib.fastapi import register_tortoise

from api import user, robot_info, code_server, ftp_info, login


logging.basicConfig(
    level=logging.DEBUG,
    format='[%(asctime)s] [%(levelname)s] [%(filename)s:%(lineno)s]: %(message)s',
    handlers=[
        logging.FileHandler('remote_bot_system.log'),
        logging.StreamHandler()
    ]
)


app = FastAPI()


register_tortoise(
    app,
    db_url="mysql://root:root@127.0.0.1:3306/cloud_test_bench",
    modules={"models": ["models.robot", "models.user", "models.ftp"]},
    generate_schemas=True,
    add_exception_handlers=True,
)


app.include_router(robot_info.router)
app.include_router(user.router)
app.include_router(code_server.router)
app.include_router(ftp_info.router)
app.include_router(login.router)


if __name__ == "__main__":
    pass