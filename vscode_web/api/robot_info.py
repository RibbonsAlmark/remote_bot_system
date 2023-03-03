import uuid

from fastapi import APIRouter, HTTPException
from tortoise.contrib.fastapi import HTTPNotFoundError

from models.status import Status
from models.robot import RobotInfo_Pydantic, RobotInfoIn_Pydantic, RobotInfo


router = APIRouter(tags=["robot info"])


@router.get(
    "/robot_info/", 
    response_model=RobotInfo_Pydantic, 
    responses={404: {"model": HTTPNotFoundError}},
    description="get robot info by uuid"
)
async def get_robot_info(uuid: str):
    return await RobotInfo_Pydantic.from_queryset_single(RobotInfo.get(uuid=uuid))


@router.put(
    "/robot_info/", 
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

    
@router.delete("/robot_info/", response_model=Status, responses={404: {"model": HTTPNotFoundError}})
async def delete_robot_info(uuid: str):
    deleted_count = await RobotInfo.filter(uuid=uuid).delete()
    if not deleted_count:
        raise HTTPException(status_code=404, detail=f"robot not found: uuid={uuid}")
    return Status(message=f"Deleted robot, uuid={uuid}")