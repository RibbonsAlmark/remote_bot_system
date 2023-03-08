import threading
from typing import Dict




class CodeServerContainer:
    
    def __init__(self, username):
        self.username = username
        self.data = {}

    def update_data(self, key, value):
        self.data[key] = value

    def get_data(self, key):
        return self.data.get(key, None)




class CodeServerContainerManager:
    
    _instance = None
    _lock: threading.RLock
    _code_server_containers: Dict[str, CodeServerContainer]

    def __init__(self):
        self._code_server_containers = {}
        self._lock = threading.RLock()

    @classmethod
    def get_instance(cls):
        with CodeServerContainerManager._lock:
            if cls._instance is None:
                cls._instance = CodeServerContainerManager()
        return cls._instance

    def create_container(self, username):
        new_container = CodeServerContainer(username)
        self.code_server_containers[username] = new_container
        return new_container

    def destroy_container(self, username):
        if username in self.users:
            del self.users[username]

    def get_user(self, username):
        if username in self.users:
            return self.users[username]
        return None



code_server_container_manager = CodeServerContainerManager()






# import redis
# import threading

# from code_workspace import CodeWorkspaceNode


# class CodeserverWsManager():
    
#     __codeserver_containers: dict
    
#     def __init__(self) -> None:
#         self.__codeserver_containers = {}
#         self._lock = threading.RLock()
    
#     def __get_coderserver_wsid(self, robot_uuid:str, robot_username:str):
#         return f"{robot_uuid}_{robot_username}"
        
    
#     def regist(self, robot_uuid:str, robot_username:str, codeserver_container):
#         codeserver_wsid = self.__get_coderserver_wsid(robot_uuid, robot_username)
#         with self._lock:
#             self.__codeserver_containers[codeserver_wsid] = codeserver_container
            
    
#     def remove(self, robot_uuid, robot_username):
#         pass
    
    
# codeserver_ws_manager = CodeserverWsManager()

# with codeserver_ws_manager._lock:
#     codeserver_ws_manager.regist(。。。)

# import threading

# class User:
#     def __init__(self, username):
#         self.username = username

# class Container:
#     def __init__(self):
#         self._dependencies = {}
#         self._lock = threading.RLock()

#     def add_dependency(self, name, dependency):
#         with self._lock:
#             self._dependencies[name] = dependency

#     def get_dependency(self, name):
#         with self._lock:
#             return self._dependencies[name]

# container = Container()
# container.add_dependency('user1', User('Alice'))
# container.add_dependency('user2', User('Bob'))

# # 在多线程环境下，使用读写锁保证访问共享状态的线程安全
# def worker():
#     with container._lock:
#         user1 = container.get_dependency('user1')
#         user2 = container.get_dependency('user2')
#     # 在这里使用 user1 和 user2

# threads = [threading.Thread(target=worker) for i in range(10)]
# for t in threads:
#     t.start()

# for t in threads:
#     t.join()