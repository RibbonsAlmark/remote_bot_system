import time
import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy.context import Context
from gazebo_msgs.srv import GetEntityState
from gazebo_msgs.msg import EntityState



class EntityStateGetter:
    
    _node: Node
    _client: Client
    
    def __init__(self, name:str, context:Context=None) -> None:
        if context == None:
            context = Context()
            context.init(args=[])
        self._node = Node(name, context=context)
        self._client = self._node.create_client(GetEntityState, "/gazebo/get_entity_state")
        
    def __is_service_available(self) -> bool:
        return self._client.wait_for_service(timeout_sec=1.0)
    
    def get_entity_state(self, entity_name:str, reference_frame:str="world") -> EntityState:
        if not self.__is_service_available():
            self._node.get_logger().info("Failed to get model state, service is unavailable")
            return None
        else:
            req = GetEntityState.Request()
            req.name = entity_name
            req.reference_frame = reference_frame
            future = self._client.call_async(req)
            rclpy.spin_until_future_complete(self._node, future)
            if future.result() is not None:
                entity_state:EntityState = future.result().state
                return entity_state
            else:
                self._node.get_logger().info("Failed to get model state, response is none")
                return None
            
    def __del__(self):
        self._node.destroy_node()
        
        
        

if __name__ == '__main__':
    context = Context()
    context.init(args=[], domain_id=1)
    Getter = EntityStateGetter("get_model_state_client", context)
    entity_state = Getter.get_entity_state("fishbot")
    print("[ENTITY INFO]")
    print("entity_state.name               : ", entity_state.name)
    print("entity_state.reference_frame    : ", entity_state.reference_frame)
    print("[POSE-position]")
    print("entity_state.pose.position.x    : ", entity_state.pose.position.x)
    print("entity_state.pose.position.y    : ", entity_state.pose.position.y)
    print("entity_state.pose.position.z    : ", entity_state.pose.position.z)
    print("[POSE-orientation]")
    print("entity_state.pose.orientation.x : ", entity_state.pose.orientation.x)
    print("entity_state.pose.orientation.y : ", entity_state.pose.orientation.y)
    print("entity_state.pose.orientation.z : ", entity_state.pose.orientation.z)
    print("entity_state.pose.orientation.w : ", entity_state.pose.orientation.w)
    print("[TWIST-angular]")
    print("entity_state.twist.angular.x    : ", entity_state.twist.angular.x)
    print("entity_state.twist.angular.y    : ", entity_state.twist.angular.y)
    print("entity_state.twist.angular.z    : ", entity_state.twist.angular.z)
    print("[TWIST-linear]")
    print("entity_state.twist.linear.x     : ", entity_state.twist.linear.x)
    print("entity_state.twist.linear.y     : ", entity_state.twist.linear.y)
    print("entity_state.twist.linear.y     : ", entity_state.twist.linear.z)
    time.sleep(1)