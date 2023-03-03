import redis
import threading


class CodeserverWorkspaceManager:
    
    
    def __init__(self) -> None:
        pass
    
    
    def create_workspace(self):
        pass
    
    
    def delete_workspace(self):
        pass


class CodeserverWsManager():
    
    __codeserver_containers: dict
    
    def __init__(self) -> None:
        self.__codeserver_containers = {}
        self._lock = threading.RLock()
    
    def __get_coderserver_wsid(self, robot_uuid:str, robot_username:str):
        return f"{robot_uuid}_{robot_username}"
        
    
    def regist(self, robot_uuid:str, robot_username:str, codeserver_container):
        codeserver_wsid = self.__get_coderserver_wsid(robot_uuid, robot_username)
        with self._lock:
            self.__codeserver_containers[codeserver_wsid] = codeserver_container
            
    
    def remove(self, robot_uuid, robot_username):
        pass
    
    
codeserver_ws_manager = CodeserverWsManager()

# with codeserver_ws_manager._lock:
#     codeserver_ws_manager.regist(。。。)