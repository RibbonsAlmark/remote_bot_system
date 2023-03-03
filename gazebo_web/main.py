import time
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
from gazebo_msgs.msg import EntityState

def get_entity_state(args=None):
    rclpy.init(args=args)
    node = Node("get_model_state_client")
    client = node.create_client(GetEntityState, "/gazebo/get_entity_state")
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Service not available, waiting again...")
        
    req = GetEntityState.Request()
    req.name = "fishbot"
    req.reference_frame = "world"
    
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        entity_state:EntityState = future.result().state
        
        print("entity_state.name               : ", entity_state.name)
        print("entity_state.reference_frame    : ", entity_state.reference_frame)
        
        print("[POSE]")
        
        print("entity_state.pose.position.x    : ", entity_state.pose.position.x)
        print("entity_state.pose.position.y    : ", entity_state.pose.position.y)
        print("entity_state.pose.position.z    : ", entity_state.pose.position.z)
        
        print("entity_state.pose.orientation.x : ", entity_state.pose.orientation.x)
        print("entity_state.pose.orientation.y : ", entity_state.pose.orientation.y)
        print("entity_state.pose.orientation.z : ", entity_state.pose.orientation.z)
        print("entity_state.pose.orientation.w : ", entity_state.pose.orientation.w)
        
        print("[TWIST]")
        
        print("entity_state.twist.angular.x    : ", entity_state.twist.angular.x)
        print("entity_state.twist.angular.y    : ", entity_state.twist.angular.y)
        print("entity_state.twist.angular.z    : ", entity_state.twist.angular.z)
        
        print("entity_state.twist.linear.x     : ", entity_state.twist.linear.x)
        print("entity_state.twist.linear.y     : ", entity_state.twist.linear.y)
        print("entity_state.twist.linear.y     : ", entity_state.twist.linear.z)
    else:
        node.get_logger().error("Failed to get model state")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    get_entity_state()
        