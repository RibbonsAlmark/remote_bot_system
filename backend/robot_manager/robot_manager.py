from typing import Dict


class RoobotEntity:
    
    def __init__(self) -> None:
        pass
    
    def __del__(self) -> None:
        pass



class RobotManager:
    
    _robot_dict: Dict[str, RoobotEntity]
    
    def __init__(self) -> None:
        pass
    
    
    
robot_manager = RobotManager()