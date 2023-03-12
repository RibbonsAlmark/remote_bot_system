from fastapi import APIRouter, HTTPException
from tortoise.contrib.fastapi import HTTPNotFoundError

from pydantic import BaseModel
from models.status import Status
from models.user import UserInfo

from fastapi import HTTPException, status, Header, Depends

from features.user import user_manager
from features.token import generate_token, verify_token, decode_token

ACCESS_TOKEN_EXPIRE_MINUTES = 30


router = APIRouter(tags=["login"])


class Form_login(BaseModel):
    user_uuid: str
    password: str
    

class LoginStatus(Status):
    token: str = ""

async def get_token_header(Authorization: str = Header(...)):
    if not Authorization:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Token missing")
    return Authorization

async def verify_x_token(token: str = Depends(get_token_header)):
    if verify_token(token):
        token_data = decode_token(token)
        return token_data
    else:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Token invalid or expired")

@router.post("/login/" , response_model=LoginStatus)
async def create_user(form: Form_login):
    username = form.user_uuid
    password = form.password
    user_info = await UserInfo.get_or_none(username=username, password=password)
    if user_info is None:
        raise HTTPException(status_code=401, detail="Incorrect username or password")
    user_manager.create(user_info)
    token = generate_token(
        data = {
            "username": user_info.username,
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
    success = user_manager.remove(user_info.uuid)
    if success:
        return Status(success=True, message=f"user '{username}' logout")
    else:
        return Status(success=False, message=f"user '{username}' logout failed")