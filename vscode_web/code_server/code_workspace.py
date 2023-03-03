import docker
import subprocess as sub
from subprocess import DEVNULL, STDOUT


class CodeWorkspaceNode:
    
    __constructed: bool
    __ftp_mount_point: str
    __code_server_container_id: str
    
    def __init__(self) -> None:
        self.__constructed = False
        
    def construct(
        self, 
        ftp_service_ip: str, 
        ftp_service_port: str, 
        ftp_user_name: str, 
        ftp_password: str, 
        bot_workspace: str, 
        ftp_mount_point: str, 
        container_name: str, 
        docker_mount_point: str, 
        port: int, 
        password: str
    ) -> int:
        if self.__constructed:
            return 0
        else:
            pass
        # set parameters
        self.__ftp_mount_point = ftp_mount_point
        # mount ftp path
        self.__unmount_ftp_path(ftp_mount_point)
        err = self.__mount_ftp_path(
            ftp_service_ip, ftp_user_name, ftp_password, 
            bot_workspace, ftp_mount_point, ftp_service_port
        )
        if err:
            return err
        else:
            pass
        # create code server container
        self.__create_code_server_container(
            container_name, ftp_mount_point, 
            docker_mount_point, port, password
        )
        self.__constructed = True
        return 0
        
    def destory(self) -> None:
        # remove code server container
        docker_client = docker.from_env()
        docker_client.containers.get(self.__code_server_container_id).remove(force=True)
        docker_client.close()
        # unmount ftp path
        self.__unmount_ftp_path(self.__ftp_mount_point)
        # reset self.__constructed
        self.__constructed = False
        
    def __exec_sys_cmd(self, cmd, time_wait = 2, stdout=DEVNULL, stderr=STDOUT) -> int:
        sub_process = sub.Popen(cmd, shell=True, stdout=stdout, stderr=stderr)
        sub_process.wait(time_wait)
        err = sub_process.poll()
        return err
        
    def __mount_ftp_path(self, ip, user_name, pass_word, src_path, mount_point, port) -> int:
        cmd = f"curlftpfs -o rw,allow_other ftp://{user_name}:{pass_word}@{ip}:{port}{src_path} {mount_point}"
        err = self.__exec_sys_cmd(cmd)
        return err
    
    def __unmount_ftp_path(self, mount_point) -> int:
        cmd = f"fusermount -u {mount_point}"
        err = self.__exec_sys_cmd(cmd)
        return err
        
    def __create_code_server_container(
        self, 
        container_name: str, 
        ftp_mount_point: str, 
        docker_mount_point: str, 
        port: int,
        password: str
    ) -> None:
        docker_client = docker.from_env()
        self.__code_server_container_id = docker_client.containers.run(
            name = container_name,
            image = "lscr.io/linuxserver/code-server:latest",
            volumes = [f"{ftp_mount_point}:{docker_mount_point}"],
            environment = [f"PASSWORD={password}"],
            ports = {"8443/tcp": port},
            privileged = True,
            detach = True
        ).id
        docker_client.close()