import time
import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy.context import Context
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.srv._set_entity_state import SetEntityState_Request
from gazebo_msgs.srv._set_entity_state import SetEntityState_Response
from gazebo_msgs.msg import EntityState



class EntityStateSetter:
    
    _node: Node
    _client: Client
    
    def __init__(self, name:str, context:Context=None) -> None:
        rclpy.init()
        if context == None:
            context = Context()
            context.init(args=[])
        self._node = Node(name, context=context)
        self._client = self._node.create_client(SetEntityState, "/gazebo/set_entity_state")
        
    def __is_service_available(self) -> bool:
        return self._client.wait_for_service(timeout_sec=1.0)
    
    def set_entity_state(self, entity_state:EntityState) -> bool:
        if not self.__is_service_available():
            self._node.get_logger().info("Failed to set model state, service is unavailable")
            return False
        else:
            req:SetEntityState_Request = SetEntityState.Request()
            req.state = entity_state
            future = self._client.call_async(req)
            rclpy.spin_until_future_complete(self._node, future)
            response:SetEntityState_Response = future.result()
            if response is not None:
                return response.success
            else:
                self._node.get_logger().info("Failed to set model state, response is none")
                return False
            
    def __del__(self):
        self._node.destroy_node()
        rclpy.shutdown()
        
        
        

if __name__ == '__main__':
    entity_state = EntityState()
    # [entity info]
    entity_state.name = "fishbot"
    entity_state.reference_frame = "world"
    # [POSE-position]
    entity_state.pose.position.x = float(1)
    entity_state.pose.position.y = float(1)
    entity_state.pose.position.z = float(0)
    # [POSE-orientation]
    entity_state.pose.orientation.x = 3.4219547762887107e-07
    entity_state.pose.orientation.y = 7.119654789318185e-06
    entity_state.pose.orientation.z = -4.3762917438959166e-05
    entity_state.pose.orientation.w = 0.9999999990170003
    # [TWIST-angular]
    entity_state.twist.angular.x = -3.6527213917676593e-09
    entity_state.twist.angular.y = -3.600580672948667e-05
    entity_state.twist.angular.z = -6.471731929582824e-06
    # [TWIST-linear]
    entity_state.twist.linear.x = -6.892908724728018e-07
    entity_state.twist.linear.y = -1.3003083800180638e-07
    entity_state.twist.linear.z = 7.191468091309751e-07
    
    Setter = EntityStateSetter("set_model_state_client")
    success = Setter.set_entity_state(entity_state)
    print(success)
    