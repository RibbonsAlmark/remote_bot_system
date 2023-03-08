import logging
from fastapi import FastAPI
from tortoise.contrib.fastapi import register_tortoise

from api import robot_info, user_info, code_server, ftp_info


logging.basicConfig(
    level=logging.DEBUG,
    format='[%(asctime)s] [%(levelname)s]: %(message)s',
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
app.include_router(user_info.router)
app.include_router(code_server.router)
app.include_router(ftp_info.router)


if __name__ == "__main__":
    pass

# class CodeWorkspaceManager:
    
#     def __init__(self) -> None:
#         pass
    
#     def create_code_workspace_node(
#         self,
#         bot_id: str,
#         user_name: str,
#     ):
#         new_node = CodeWorkspaceNode()
#         # bot_id => {
#         #     ftp_service_ip(bot_ip),
#         #     ftp_service_port(bot_pswd),
#         #     ftp_user_name,
#         #     ftp_password,
#         #     bot_workspace
#         # }
        
#         # ftp_mount_point
#         # user_name
#         # container_name
#         # docker_mount_point
#         # port
#         # password
        
#         # worker = CodeWorkspaceNode(
#         #     ftp_service_ip, 
#         #     ftp_service_port, 
#         #     ftp_user_name, 
#         #     ftp_password, 
#         #     bot_workspace, 
#         #     ftp_mount_point,         #     container_name, 
#         #     docker_mount_point, 
#         #     port,
#         #     password
#         # )
#         pass
    
#     def __del__(self):
#         pass
    
    
# from tortoise import Tortoise, fields, run_async
# from tortoise.models import Model


# class RobotInfo(Model):
    
#     id = fields.IntField(pk=True)
#     uuid = fields.CharField(max_length=255, unique=True)
#     name = fields.CharField(max_length=255, null=True)
#     type = fields.CharField(max_length=64)
#     ip = fields.CharField(max_length=64, null=True)
#     ftp_service_port = fields.IntField(null=True)
#     ftp_username = fields.CharField(max_length=128, null=True)
#     ftp_password = fields.CharField(max_length=128, null=True)
#     ftp_workspace = fields.CharField(max_length=512, null=True)
    
#     class Meta:
#         table = "robot_info"
        
#     def __str__(self):
#         return self.name

    
# async def run():
#     await Tortoise.init(
#         db_url = "mysql://root:root@127.0.0.1:3306/cloud_test_bench",
#         modules={"models": ["__main__"]}
#     )
#     await Tortoise.generate_schemas()
#     print(await RobotInfo.get(id=1).values())

# if __name__ == "__main__":
#     run_async(run())
    
    
    
#     # user_name = "setsuna"
#     # bot_name = "mock_bot_001"
    
#     # ftp_service_ip = "192.168.124.137"
#     # ftp_service_port = "21"
#     # ftp_user_name = "haro"
#     # ftp_password = "1"
#     # bot_workspace = "/Documents/bot_workspace"
#     # ftp_mount_point = "/mnt/ftp/mock_bot_workspace"
#     # container_name = f"{user_name}.{bot_name}"
#     # ftp_mount_point = "/mnt/ftp/mock_bot_workspace"
#     # docker_mount_point = "/home/coder/project"
#     # port = 9080
#     # password = "12345"
    
#     # node = CodeWorkspaceNode()
#     # err = node.construct(
#     #     ftp_service_ip, 
#     #     ftp_service_port, 
#     #     ftp_user_name, 
#     #     ftp_password, 
#     #     bot_workspace, 
#     #     ftp_mount_point, 
#     #     container_name, 
#     #     docker_mount_point, 
#     #     port,
#     #     password
#     # )
    
#     # import pickle
#     # pkd_worker = pickle.dumps(node)
    
#     # time.sleep(10)
#     # node.destory()
    
    
#     while(1): time.sleep(100)