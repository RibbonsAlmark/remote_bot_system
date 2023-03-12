import logging
import threading
import asyncio
from typing import Dict

from models.robot import RobotInfo
from models.user import UserInfo
from models.ftp import FtpInfo

from ftp import ftp_mount_point_manager
from vscode import code_server_container_manager




logging.basicConfig(
    level=logging.DEBUG,
    format='[%(asctime)s] [%(levelname)s]: %(message)s',
    handlers=[
        logging.FileHandler('remote_bot_system.log'),
        logging.StreamHandler()
    ]
)




class User:
    
    __uuid: str
    
    def __init__(self, uuid:str) -> None:
        self.__uuid = uuid
        logging.info(f"user entity initialized, user uuid: [{uuid}]")
        
    async def apply_robot(self, robot_uuid, robot_username):
        pass
        
    def release(self):
        pass
    
    
    

class UserManager:
    
    __lock: threading.RLock
    __users: Dict[str, User]
    
    def __init__(self) -> None:
        self.__users = {}
        self.__lock = threading.RLock()
        logging.info("user manager initialized")
        
        
    def create_user_entity(self, user_uuid) -> User:
        try:
            new_user = self.__users[user_uuid] = User(user_uuid)
            with self.__lock:
                self.__users[user_uuid] = new_user
            return new_user
        except Exception as e:
            logging.warning(f"create user [{user_uuid}] failed: {e}")
            return None
    
    def get_user_entity(self, user_uuid) -> User:
        if user_uuid in self.__users:
            return self.__users[user_uuid]
        else:
            logging.info(f"get user failed, user manager could not find user-[{user_uuid}]")
            return None
            
    def release_user_entity(self, user_uuid) -> None:
        user = self.get_user_entity(user_uuid)
        if user is not None:
            user.release()
            del self.__users[user_uuid]
        else:
            logging.info(f"cannot find user-[{user_uuid}], stop release process")
        
    
    
    
    
if __name__ == "__main__":
    pass