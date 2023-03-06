import rclpy
from rclpy.node import Node
from rclpy.context import Context
from rclpy.client import Client
from gazebo_msgs.srv import GetEntityState
from gazebo_msgs.srv._get_entity_state import GetEntityState_Request
from gazebo_msgs.srv._get_entity_state import GetEntityState_Response
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.srv._set_entity_state import SetEntityState_Request
from gazebo_msgs.srv._set_entity_state import SetEntityState_Response
from gazebo_msgs.msg import EntityState


class GetStateNode(Node):
    
    _client: Client
    _request: GetEntityState_Request
    
    def __init__(
        self, 
        node_name:str="state_getter", 
        context:Context=None
    ) -> None:
        if context == None:
            context = Context()
            context.init(args=[])
        super().__init__(node_name, context=context)
        self.get_logger().info(f"node {node_name} init..")
        self._client = self.create_client(GetEntityState, "/gazebo/get_entity_state")
        self._request = GetEntityState.Request()
        
    def __is_service_available(self) -> bool:
        return self._client.wait_for_service(timeout_sec=1.0)
    
    def get_entity_state(self, entity_name:str, reference_frame:str="world", ) -> EntityState:
        if not self.__is_service_available():
            self.get_logger().info("Failed to get model state, service is unavailable")
            return None
        else:
            self._request.name = entity_name
            self._request.reference_frame = reference_frame
            future = self._client.call_async(self._request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                entity_state:EntityState = future.result().state
                return entity_state
            else:
                self.get_logger().info("Failed to get model state, response is none")
                return None


class SetStateNode(Node):
    
    _client: Client
    _request: SetEntityState_Request
    
    def __init__(
        self, 
        node_name:str="state_setter", 
        context:Context=None
    ) -> None:
        if context == None:
            context = Context()
            context.init(args=[])
        super().__init__(node_name, context=context)
        self.get_logger().info(f"node {node_name} init..")
        self._client = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self._request = SetEntityState.Request()
        
    def __is_service_available(self) -> bool:
        return self._client.wait_for_service(timeout_sec=1.0)
    
    def set_entity_state(self, entity_state:EntityState) -> bool:
        if not self.__is_service_available():
            self.get_logger().info("Failed to set model state, service is unavailable")
            return False
        else:
            self._request.state = entity_state
            future = self._client.call_async(self._request)
            rclpy.spin_until_future_complete(self, future)
            response:SetEntityState_Response = future.result()
            if response is not None:
                return response.success
            else:
                self.get_logger().info("Failed to set model state, response is none")
                return False
            
            
def main(args=None):
    entity_name = "fishbot"
    local_domain_id = 1
    remote_domain_id = 0
    
    rclpy.init()
    
    context_getter = Context()
    context_getter.init(args=[], domain_id=local_domain_id)
    getter_node = GetStateNode(context=context_getter)
    
    context_setter = Context()
    context_setter.init(args=[], domain_id=remote_domain_id)
    setter_node = SetStateNode(context=context_setter)
    
    import time
    while True:
        entity_state = getter_node.get_entity_state(entity_name)
        if entity_state  is not None:
            entity_state.name = entity_name
            success = setter_node.set_entity_state(entity_state)
            print(success)
        time.sleep(0.02)
        
    rclpy.spin(node)
    rclpy.shutdown()