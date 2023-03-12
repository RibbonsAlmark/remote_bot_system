import jwt
from datetime import datetime, timedelta

# jwt secret key
secret_key = 'my_secret_key'
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30

# generate JWT token
def generate_token(data, expires_minutes=ACCESS_TOKEN_EXPIRE_MINUTES) -> str:
    exp_time = datetime.utcnow() + timedelta(minutes=expires_minutes)
    data.update({'exp': exp_time, 'iss': 'my_issuer'})
    token = jwt.encode(data, secret_key, algorithm=ALGORITHM)
    return token.decode('utf-8')

# verify JWT token
def verify_token(token) -> bool:
    try:
        decoded = jwt.decode(token, secret_key, algorithms=[ALGORITHM], issuer='my_issuer')
        return True
    except:
        return False

# decode JWT token
def decode_token(token) -> bool:
    try:
        decoded = jwt.decode(token, secret_key, algorithms=[ALGORITHM], issuer='my_issuer')
        return decoded
    except:
        return None
    
    
if __name__ == "__main__":
    data = {
        "username": "setsuna"
    }
    
    token = generate_token(data)
    valid = verify_token(token)
    data = decode_token(token)
    print(data)