import uuid

from fastapi import APIRouter, HTTPException
from tortoise.contrib.fastapi import HTTPNotFoundError

from models.status import Status
from models.user import UserInfo_Pydantic, UserInfoIn_Pydantic, UserInfo


router = APIRouter(tags=["user info"])


@router.get(
    "/user/", 
    response_model=UserInfo_Pydantic, 
    responses={404: {"model": HTTPNotFoundError}}
)
async def get_user(uuid: str):
    return await UserInfo_Pydantic.from_queryset_single(UserInfo.get(uuid=uuid))


@router.put(
    "/user/", 
    response_model=UserInfo_Pydantic, 
    responses={404: {"model": HTTPNotFoundError}}
)
async def update_user(uuid: str, robot_info: UserInfoIn_Pydantic):
    await UserInfo.filter(uuid=uuid).update(**robot_info.dict(exclude_unset=True, exclude={"uuid"}, exclude_none=True))
    return await UserInfo_Pydantic.from_queryset_single(UserInfo.get(uuid=uuid))


@router.post("/user/" , response_model=UserInfo_Pydantic)
async def create_user(robot_info: UserInfoIn_Pydantic):
    robot_info_dict = robot_info.dict(exclude_unset=True, exclude_none=True)
    robot_info_dict["uuid"] = uuid.uuid1()
    robot_info_obj = await UserInfo.create(**robot_info_dict)
    return await UserInfo_Pydantic.from_tortoise_orm(robot_info_obj)

    
@router.delete("/user/", response_model=Status, responses={404: {"model": HTTPNotFoundError}})
async def delete_user(uuid: str):
    deleted_count = await UserInfo.filter(uuid=uuid).delete()
    if not deleted_count:
        raise HTTPException(status_code=404, detail=f"robot not found: uuid={uuid}")
    return Status(message=f"Deleted robot, uuid={uuid}")