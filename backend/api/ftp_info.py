import uuid
from typing import List

from fastapi import APIRouter, HTTPException
from tortoise.contrib.fastapi import HTTPNotFoundError

from models.status import Status
from models.ftp import FtpInfo_Pydantic, FtpInfoIn_Pydantic_Post, FtpInfoIn_Pydantic_Get, FtpInfoIn_Pydantic_Del, FtpInfo


router = APIRouter(tags=["ftp info"])


@router.get(
    "/ftp_info/", 
    response_model=List[FtpInfo_Pydantic], 
    responses={404: {"model": HTTPNotFoundError}},
    description="get ftp info by robot uuid & ftp user name"
)
async def get_ftp_info(ftp_info: FtpInfoIn_Pydantic_Get):
    return await FtpInfo_Pydantic.from_queryset(FtpInfo.filter(**ftp_info.dict(exclude_unset=True)))


# @router.put(
#     "/ftp_info/", 
#     response_model=FtpInfo_Pydantic, 
#     responses={404: {"model": HTTPNotFoundError}}
# )
# async def update_ftp_info(uuid: str, robot_info: FtpInfoIn_Pydantic):
#     await FtpInfo.filter(uuid=uuid).update(**robot_info.dict(exclude_unset=True))
#     return await FtpInfo_Pydantic.from_queryset_single(FtpInfo.get(uuid=uuid))


@router.post("/ftp_info/" , response_model=FtpInfo_Pydantic)
async def create_ftp_info(ftp_info: FtpInfoIn_Pydantic_Post):
    ftp_info_dict = ftp_info.dict(exclude_unset=True)
    ftp_info_dict["mount_point"] = "/mnt/ftp/mock_bot_workspace"
    ftp_info_obj = await FtpInfo.create(**ftp_info_dict)
    return await FtpInfo_Pydantic.from_tortoise_orm(ftp_info_obj)

    
@router.delete("/ftp_info/", response_model=Status, responses={404: {"model": HTTPNotFoundError}})
async def delete_ftp_info(ftp_info: FtpInfoIn_Pydantic_Del):
    deleted_count = await FtpInfo.get(**ftp_info.dict(exclude_unset=True)).delete()
    if not deleted_count:
        raise HTTPException(status_code=404, detail=f"ftp info not found: robot_uuid={ftp_info.robot_uuid}, ftp username={ftp_info.username}")
    return Status(message=f"Deleted ftp info: robot_uuid uuid={ftp_info.robot_uuid}, ftp username={ftp_info.username}")