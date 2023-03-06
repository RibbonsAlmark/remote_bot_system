import rclpy
from rclpy.node import Node

def main():
    rclpy.init()

    # 创建属于domain_id=0的Node实例
    context0 = rclpy.Context()
    node0 = Node('node0', context=context0)

    # 创建属于domain_id=1的Node实例
    context1 = rclpy.Context()
    context1.init(args=[], domain_id=1)
    node1 = Node('node1', context=context1)

    # 在两个节点中分别执行相应的操作
    # ...

    node0.destroy_node()
    node1.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()