import os
import logging
import threading
import ftplib
from typing import Dict

import subprocess as sub
from subprocess import DEVNULL, STDOUT




class FtpMountPoint:
    
    __host: str
    __port: int
    __username: str
    __password: str
    __remote_dir: str
    __mount_point: str
    
    def __init__(
        self,
        host: str, 
        port: int, 
        username: str, 
        password: str, 
        remote_dir: str, 
        mount_point: str, 
    ) -> None:
        self.__host = host
        self.__port = port
        self.__username = username
        self.__password = password
        self.__remote_dir = remote_dir
        self.__mount_point = mount_point
        logging.info(f"FtpMounter initialized, host:{host}, port:{port}, username:{username}, remote_dir:{remote_dir}, mount_point:{mount_point}")
        
    def __ftp_service_available(self) -> bool:
        try:
            with ftplib.FTP(self.__host) as ftp:
                ftp.login(user=self.__username, passwd=self.__password)
                ftp.cwd('/')
                logging.debug("ftp service is available now")
                return True
        except Exception as e:
            logging.debug(f"ftp service is unavailable now, error info :{e}")
            return False
        
    def __exec_sys_cmd(self, cmd, time_wait = 2, stdout=DEVNULL, stderr=STDOUT) -> int:
        logging.debug(f"execute system cmd by sub.Popen, cmd: [{cmd}]")
        try:
            sub_process = sub.Popen(cmd, shell=True, stdout=stdout, stderr=stderr)
            sub_process.wait(time_wait)
        except Exception as e:
            logging.warning(e)
            return 1
        err = sub_process.poll()
        if err:
            logging.debug(f"failed to execute system cmd: {cmd}")
        return err
        
    def mount(self) -> int:
        if self.__ftp_service_available():
            logging.info(f"mount ftp folder, host:{self.__host} port:{self.__port} username:{self.__username} remote_dir:{self.__remote_dir} mount_point:{self.__mount_point}")
            cmd = f"curlftpfs -o rw,allow_other ftp://{self.__username}:{self.__password}@{self.__host}:{self.__port}{self.__remote_dir} {self.__mount_point}"
            err = self.__exec_sys_cmd(cmd)
            return err
        else:
            logging.info("ftp service unavailable, do not execute mount ftp folder action")
            return 1
        
    def unmount(self) -> int:
        if self.__ftp_service_available():
            logging.info(f"unmount ftp folder, host:{self.__host} port:{self.__port} username:{self.__username} remote_dir:{self.__remote_dir} mount_point:{self.__mount_point}")
            cmd = f"fusermount -u {self.__mount_point}"
            err = self.__exec_sys_cmd(cmd)
            return err
        else:
            logging.info("ftp service unavailable, do not execute unmount ftp folder action")
            
    def __del__(self):
        self.unmount()
    
    
    

class FtpMountPointManager:
    
    __mount_point_dict: Dict[str, FtpMountPoint]
    __lock: threading.RLock
    __mount_point_root: str
    
    def __init__(self, mount_point_root:str) -> None:
        self.__mount_point_root = mount_point_root
        self.__clear_mount_point_root()
        self.__mount_point_dict = {}
        self.__lock = threading.RLock()
        
    def __clear_mount_point_root(self):
        for folder in os.listdir(self.__mount_point_root):
            mount_point = os.path.join(self.__mount_point_root, folder)
            logging.debug(f"unmount: {mount_point}")
            cmd = f"fusermount -u {mount_point}"
            try:
                sub_process = sub.Popen(cmd, shell=True, stdout=DEVNULL, stderr=STDOUT)
                sub_process.wait(2)
            except Exception as e:
                pass
            err = sub_process.poll()
        
    def create_mount_point(
        self, 
        host: str, 
        port: int, 
        username: str, 
        password: str, 
        remote_dir: str, 
        mount_point: str
    ) -> int:
        with self.__lock:
            if mount_point in self.__mount_point_dict:
                logging.debug(f"ftp mount point [{mount_point}] already mounted, skip create")
                return 1
            self.__mount_point_dict[mount_point] = FtpMountPoint(host, port, username, password, remote_dir, mount_point)
            err = self.__mount_point_dict[mount_point].mount()
            if err:
                logging.debug(f"failed to create mounter:[{mount_point}]")
                del self.__mount_point_dict[mount_point]
            else:
                pass
            return err
    
    def release_mount_point(self, mount_point:str) -> None:
        with self.__lock:
            if mount_point in self.__mount_point_dict:
                logging.debug(f"release ftp mount point [{mount_point}]")
                self.__mount_point_dict[mount_point].unmount()
                del self.__mount_point_dict[mount_point]
            
    def __del__(self):
        self.__clear_mount_point_root()
    
    
    
    
# class FtpMountManager:
    
#     __redis_client: redis.Redis
#     __hash_name_in_redis: str
    
#     def __init__(
#         self, redis_host:str, redis_port:int, redis_password:str, 
#         hash_name_in_redis:str="ftpMounters"
#     ) -> None:
#         self.__redis_client = redis.Redis(host=redis_host, port=redis_port, password=redis_password)
#         self.__hash_name_in_redis = hash_name_in_redis
#         logging.info("ftp mount manager initialized")
    
#     def __set_mounter(self, mounter_name:str, mounter:FtpMounter) -> None:
#         logging.debug(f"set mounter [{mounter_name}] to redis")
#         self.__redis_client.hset(self.__hash_name_in_redis, mounter_name, pickle.dumps(mounter))
        
#     def __get_mounter(self, mounter_name:str) -> FtpMounter:
#         logging.debug(f"get mounter [{mounter_name}] from redis")
#         mounter_pickled = self.__redis_client.hget(self.__hash_name_in_redis, mounter_name)
#         if mounter_pickled is not None:
#             logging.debug(f"mounter [{mounter_name}] dose not exesit redis")
#             return pickle.loads(mounter_pickled)
#         else:
#             return None
        
#     def __del_mounter(self, mounter_name:str) -> None:
#         logging.debug(f"del mounter [{mounter_name}] from redis")
#         self.__redis_client.hdel(self.__hash_name_in_redis, mounter_name)
        
#     def get_mounter_name(self, host:str, port:int, username:str) -> str:
#         return f"ftpMounter-{host}-{port}-{username}"
        
#     def create_mounter(
#         self, 
#         host: str, 
#         port: int, 
#         username: str, 
#         password: str, 
#         remote_dir: str, 
#         mount_point: str, 
#     ) -> None:
#         mounter_name = self.get_mounter_name(host, port, username)
#         if self.__redis_client.hget(self.__hash_name_in_redis, mounter_name) is not None:
#             logging.debug(f"mounter [{mounter_name}] already exeist, skip create")
#         else:
#             pass
#         new_mounter = FtpMounter(host, port, username, password, remote_dir, mount_point)
#         err = new_mounter.mount()
#         if err:
#             logging.debug(f"failed to create mounter:[{mounter_name}]")
#         else:
#             self.__set_mounter(mounter_name, new_mounter)
        
#     def destroy_mounter(self, host:str, port:int, username:str):
#         mounter_name = self.get_mounter_name(host, port, username)
#         logging.debug(f"try to destroy mounter [{mounter_name}]")
#         mounter:FtpMounter = self.__get_mounter(mounter_name)
#         if mounter is not None:
#             mounter.unmount()
#             self.__del_mounter(mounter_name)
#         else:
#             logging.debug(f"mounter [{mounter_name}] dose not exeist, skip destroy")




ftp_mount_point_manager = FtpMountPointManager("/mnt/ftp")




if __name__ == "__main__":
    logging.basicConfig(
        level=logging.DEBUG,
        format='[%(asctime)s] [%(levelname)s]: %(message)s',
        handlers=[
            logging.FileHandler('remote_bot_system.log'),
            logging.StreamHandler()
        ]
    )
    
    mmount_point_root = "/mnt/ftp"
    ftp_host = "192.168.124.139"
    ftp_port = 21
    ftp_username = "setsuna"
    ftp_password = "1"
    remote_dir = "/"
    mmount_point = "/mnt/ftp/mock_bot_workspace"
    ftp_mount_point_manager = FtpMountPointManager(mmount_point_root)
    
    import time
    ftp_mount_point_manager.create_mount_point(ftp_host, ftp_port, ftp_username, ftp_password, remote_dir, mmount_point)
    time.sleep(10)
    ftp_mount_point_manager.release_mount_point(mmount_point)
    while True:
        time.sleep(100)