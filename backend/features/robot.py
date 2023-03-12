import sys 
sys.path.append("..")

import logging
import threading
from typing import Dict, Union

from models.robot import RobotInfo
from models.ftp import FtpInfo

from ftp import ftp_mount_point_manager
from vscode import code_server_container_manager

from ftp import FtpMountPoint
from vscode import CodeServerContainer




class Robot:
    
    __robot_info: RobotInfo
    __ftp_info: FtpInfo
    __entity_id: str
    
    __user_uuid: str = None
    __ftp_mount_point: FtpMountPoint = None
    __code_server_container: CodeServerContainer = None
    
    def __init__(self, robot_info: RobotInfo, ftp_info:FtpInfo) -> None:
        self.__robot_info = robot_info
        self.__ftp_info = ftp_info
        self.__entity_id = RobotManager.get_entity_id(robot_info.uuid, ftp_info.username)
        self.mount_workspace(ftp_info)
        
    def set_user(self, user_uuid:str) -> None:
        self.__user_uuid = user_uuid
        
    def remove_user(self) -> None:
        self.__user_uuid = None
        
    @property
    def user_uuid(self):
        return self.__user_uuid
    
    @property
    def entity_id(self):
        return self.__entity_id
        
    def __workspace_mounted(self):
        if self.__ftp_mount_point is None:
            return False
        elif ftp_mount_point_manager.get(self.__ftp_mount_point.path) is None:
            return False
        else:
            return True
    
    def mount_workspace(self, ftp_info: FtpInfo) -> bool:
        if self.__workspace_mounted():
            logging.debug(f"robot entity [{self.__entity_id}] workspace already mounted, terminate mount process")
            return True
        else:
            self.__ftp_mount_point = ftp_mount_point_manager.create_mount_point(
                host=self.__robot_info.ip,
                port=self.__ftp_info.port,
                username=self.__ftp_info.username,
                password=self.__ftp_info.password,
                remote_dir=self.__ftp_info.remote_dir,
                mount_point=self.__ftp_info.mount_point
            )
        if self.__workspace_mounted():
            logging.debug(f"robot entity [{self.__entity_id}] mount workspace success")
            return True
        else:
            logging.debug(f"robot entity [{self.__entity_id}] mount workspace failed")
            return False
    
    def unmount_workspace(self) -> None:
        if self.__workspace_mounted():
            ftp_mount_point_manager.release_mount_point(self.__ftp_mount_point.path)
            logging.info(f"robot entity [{self.__entity_id}] workspace unmounted")
        else:
            logging.debug(f"robot entity [{self.__entity_id}] workspace not mounted, terminate unmount process")
            
    def code_server_container_created(self) -> bool:
        if self.__code_server_container is None:
            return False
        elif code_server_container_manager.get(self.__code_server_container.id) is None:
            return False
        else:
            return True
    
    def create_code_server_container(self, password:str, workspace:str, port:int=None, name:str=None) -> bool:
        if self.code_server_container_created():
            logging.debug(f"robot entity [{self.__entity_id}] already created code server container, terminate create process")
            return True
        else:
            self.__code_server_container = code_server_container_manager.create_container(
                password=password,
                host_dir=self.__ftp_info.mount_point,
                container_dir=workspace,
                port=port,
                container_name=name
            )
            container_created =  self.code_server_container_created()
            if container_created:
                logging.debug(f"code server container created by robot entity [{self.__entity_id}], container id: [{self.__code_server_container.id}]")
            else:
                logging.debug(f"robot entity [{self.__entity_id}] create code server container failed")
            return container_created
    
    def release_code_server_container(self) -> bool:
        if not self.code_server_container_created():
            logging.debug(f"robot entity [{self.__entity_id}] not hold code server container, terminate release process")
            return True
        else:
            container_id = self.__code_server_container.id
            container_removed = code_server_container_manager.remove_container(container_id)
            if container_removed:
                logging.debug(f"robot [{self.__entity_id}] remove code server container [{container_id}] success")
                self.__code_server_container = None
                return True
            else:
                logging.debug(f"robot [{self.__entity_id}] remove code server container [{container_id}] failed")
                return False
    
    def shutdown(self):
        if self.code_server_container_created():
            self.release_code_server_container()
        if self.__workspace_mounted():
            self.unmount_workspace()
        logging.info(f"robot {self.__entity_id} shutdown")
    
    
    
    
class RobotManager:
    
    __lock: threading.RLock
    __robots: Dict[str, Robot]
    
    def __init__(self) -> None:
        self.__robots = {}
        self.__lock = threading.RLock()
        logging.info("robot manager initialized")
    
    @classmethod
    def get_entity_id(self, robot_uuid:str, bot_username:str):
        entity_id = f"{robot_uuid}_{bot_username}"
        return entity_id
    
    def __get(self, entity_id:str) -> Union[Robot, None]:
        with self.__lock:
            if entity_id in self.__robots:
                return self.__robots[entity_id]
            else:
                return None
        
    def get(self, robot_uuid:str, robot_username:str) -> Union[Robot, None]:
        entity_id = self.get_entity_id(robot_uuid, robot_username)
        return self.__get(entity_id)
        
    def create(self, robot_info:RobotInfo, ftp_info:FtpInfo) -> Robot:
        robot_uuid = robot_info.uuid
        robot_username = ftp_info.username
        entity_id = self.get_entity_id(robot_uuid, robot_username)
        robot = self.__get(entity_id)
        if robot is not None:
            logging.debug(f"robot [{entity_id}] already exsist, terminate create process")
            return robot
        else:
            robot = Robot(robot_info, ftp_info)
            if robot is not None:
                logging.info(f"create robot [{entity_id}] success")
                with self.__lock:
                    self.__robots[entity_id] = robot
                return robot
            else:
                logging.info(f"create robot [{entity_id}] failed")
                return None
            
    def destory(self, robot_uuid:str, robot_username:str) -> None:
        entity_id = self.get_entity_id(robot_uuid, robot_username)
        robot = self.__get(entity_id)
        if robot is None:
            logging.debug(f"robot [{entity_id}] not created yet, terminate destory process")
        else:
            robot.shutdown()
            with self.__lock:
                del self.__robots[entity_id]
            logging.info(f"destory robot [{entity_id}] success")
            
    def __is_bot_allocated(self, robot:Robot) -> bool:
        if robot.user_uuid is not None:
            logging.debug(f"robot [{robot.entity_id}] was allocated, user [{robot.user_uuid}]")
            return True
        elif robot.code_server_container_created():
            logging.debug(f"robot [{robot.entity_id}] was allocated, code server container created")
            return True
        else:
            logging.debug(f"robot [{robot.entity_id}] not allocated")
            return False
    
    def allocate(self, robot_uuid:str, robot_username:str, user_uuid:str, password:str, workspace:str) -> bool:
        entity_id = self.get_entity_id(robot_uuid, robot_username)
        robot = self.__get(entity_id)
        if robot is None:
            logging.info(f"robot allocate failed, robot entity [{entity_id}] not exsist")
            return False
        elif self.__is_bot_allocated(robot):
            logging.info(f"robot allocate failed, robot entity [{entity_id}] already allocated")
            return False
        else:
            robot.set_user(user_uuid)
            container_name = f"codeServer.usr_{user_uuid}.bot_{robot_uuid}.botUsr_{robot_username}"
            robot.create_code_server_container(password, workspace, name=container_name)
            logging.info(f"robot allocate success, robot entity [{entity_id}] allocated to user [{user_uuid}]")
            return True
        
    def deallocate(self, robot_uuid:str, robot_username:str) -> bool:
        entity_id = self.get_entity_id(robot_uuid, robot_username)
        robot = self.__get(entity_id)
        if robot is None:
            logging.debug(f"robot entity [{entity_id}] not exsist, terminate robot deallocate process")
            return True
        elif not self.__is_bot_allocated(robot):
            logging.debug(f"robot entity [{entity_id}] not allocated, terminate robot deallocate process")
            return True
        else:
            code_server_released = robot.release_code_server_container()
            if not code_server_released:
                logging.info(f"robot entity [{entity_id}] deallocate failed, can not release code server container")
                return False
            else:
                robot.remove_user()
                logging.info(f"robot entity [{entity_id}] deallocate success")
                return True




if __name__ == "__main__":
    
    import time
    logging.basicConfig(
        level=logging.DEBUG,
        format='[%(asctime)s] [%(levelname)s]: %(message)s',
        handlers=[
            logging.FileHandler('remote_bot_system.log'),
            logging.StreamHandler()
        ]
    )

    robot_info = RobotInfo()
    robot_info.id = 1
    robot_info.uuid = "9d944006-bcc2-11ed-b9c9-bfe419b26b81"
    robot_info.name = "mock bot 002"
    robot_info.type = "mock bot"
    robot_info.ip = "192.168.124.139"
    
    ftp_info = FtpInfo()
    ftp_info.id = 1
    ftp_info.robot_uuid = "9d944006-bcc2-11ed-b9c9-bfe419b26b81"
    ftp_info.username = "setsuna"
    ftp_info.password = "1"
    ftp_info.remote_dir = "/"
    ftp_info.mount_point = "/mnt/ftp/mock_bot_workspace"
    ftp_info.port = 21
    
    code_server_password = "1"
    code_server_workspace = "/home/coder/project"
    
    user_uuid = "8a940796-ac48-11ed-bbda-8565430a6f0c"
    robot_uuid = robot_info.uuid
    robot_username = ftp_info.username
    
    robot_manager = RobotManager()
    robot_manager.create(robot_info, ftp_info)
    robot_manager.allocate(
        robot_uuid, robot_username, user_uuid, 
        code_server_password, 
        code_server_workspace
    )
    time.sleep(10)
    robot_manager.deallocate(robot_uuid, robot_username)
    robot_manager.destory(robot_uuid, robot_username)
    
    while True:
        time.sleep(100)
    