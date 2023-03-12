from fastapi import APIRouter, HTTPException
from tortoise.contrib.fastapi import HTTPNotFoundError

from pydantic import BaseModel
from models.status import Status
from models.user import UserInfo

from fastapi import HTTPException, Depends

from features.user import user_manager
from features.token import generate_token, verify_x_token

ACCESS_TOKEN_EXPIRE_MINUTES = 30


router = APIRouter(tags=["login"])


class Form_login(BaseModel):
    username: str
    password: str
    

class LoginStatus(Status):
    token: str = ""

@router.post("/login/" , response_model=LoginStatus)
async def create_user(form: Form_login):
    username = form.username
    password = form.password
    user_info = await UserInfo.get_or_none(username=username, password=password)
    if user_info is None:
        raise HTTPException(status_code=401, detail="Incorrect username or password")
    user_manager.create(user_info)
    token = generate_token(
        data = {
            "username": user_info.username,
            "user_uuid": user_info.uuid.__str__(),
            "role": user_info.role
        },
        expires_minutes = ACCESS_TOKEN_EXPIRE_MINUTES
    )
    return LoginStatus(success=True, message=f"user '{username}' login", token=token)

@router.post("/logout", response_model=Status)
async def logout(token_payload: dict = Depends(verify_x_token)):
    username = token_payload["username"]
    user_info = await UserInfo.get_or_none(username=username)
    if user_info is None:
        raise HTTPException(status_code=401, detail=f"user '{username}' not exist")
    success = user_manager.remove(user_info.uuid.__str__())
    if success:
        return Status(success=True, message=f"user '{username}' logout")
    else:
        return Status(success=False, message=f"user '{username}' logout failed")