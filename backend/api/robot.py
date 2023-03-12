import uuid

from fastapi import APIRouter, HTTPException
from tortoise.contrib.fastapi import HTTPNotFoundError

from pydantic import BaseModel
from models.status import Status
from models.robot import RobotInfo_Pydantic, RobotInfoIn_Pydantic, RobotInfo
from models.ftp import FtpInfo

from features.robot import robot_manager


router = APIRouter(tags=["robot"])


@router.get(
    "/robot/", 
    response_model=RobotInfo_Pydantic, 
    responses={404: {"model": HTTPNotFoundError}},
    description="get robot info by uuid"
)
async def get_robot_info(uuid: str):
    return await RobotInfo_Pydantic.from_queryset_single(RobotInfo.get(uuid=uuid))


@router.put(
    "/robot/", 
    response_model=RobotInfo_Pydantic, 
    responses={404: {"model": HTTPNotFoundError}}
)
async def update_robot_info(uuid: str, robot_info: RobotInfoIn_Pydantic):
    await RobotInfo.filter(uuid=uuid).update(**robot_info.dict(exclude_unset=True))
    return await RobotInfo_Pydantic.from_queryset_single(RobotInfo.get(uuid=uuid))


@router.post("/robot_info/" , response_model=RobotInfo_Pydantic)
async def create_robot_info(robot_info: RobotInfoIn_Pydantic):
    robot_info_dict = robot_info.dict(exclude_unset=True)
    robot_info_dict["uuid"] = uuid.uuid1()
    robot_info_obj = await RobotInfo.create(**robot_info_dict)
    return await RobotInfo_Pydantic.from_tortoise_orm(robot_info_obj)

    
@router.delete("/robot/", response_model=Status, responses={404: {"model": HTTPNotFoundError}})
async def delete_robot_info(uuid: str):
    deleted_count = await RobotInfo.filter(uuid=uuid).delete()
    if not deleted_count:
        raise HTTPException(status_code=404, detail=f"robot not found: uuid={uuid}")
    return Status(message=f"Deleted robot, uuid={uuid}")
    
    
class Form_RobotOnline(BaseModel):
    robot_uuid: str
    robot_username: str

@router.post("/robot/online" , response_model=Status)
async def robot_online(data: Form_RobotOnline):
    robot_info = await RobotInfo.get_or_none(uuid=data.robot_uuid)
    ftp_info = await FtpInfo.get_or_none(robot_uuid=data.robot_uuid, username=data.robot_username)
    if robot_info is None:
        return Status(success=False, message="robot sign in system fialed, robot not regist in system")
    elif ftp_info is None:
        return Status(success=False, message="robot sign in system fialed, robot username not regist in system")
    else:
        new_robot = robot_manager.create(robot_info, ftp_info)
        if new_robot is None:
            return Status(success=False, message="robot sign in system fialed, error happen durinig create")
        else:
            return Status(success=True, message="robot sign in system seccess")