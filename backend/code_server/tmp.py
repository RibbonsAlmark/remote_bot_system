class User:
    def __init__(self, username):
        self.username = username
        self.data = {}

    def update_data(self, key, value):
        self.data[key] = value

    def get_data(self, key):
        return self.data.get(key, None)


class UserFactory:
    _instance = None

    def __init__(self):
        self.users = {}

    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            cls._instance = UserFactory()
        return cls._instance

    def create_user(self, username):
        user = User(username)
        self.users[username] = user
        return user

    def destroy_user(self, username):
        if username in self.users:
            del self.users[username]

    def get_user(self, username):
        if username in self.users:
            return self.users[username]
        return None
    
    
# 创建用户工厂对象
factory = UserFactory.get_instance()

# 创建用户对象
user1 = factory.create_user("user1")

# 获取用户对象
user1 = factory.get_user("user1")

# 更新用户数据
user1.update_data("key", "value")

# 获取用户数据
value = user1.get_data("key")

# 销毁用户对象
factory.destroy_user("user1")