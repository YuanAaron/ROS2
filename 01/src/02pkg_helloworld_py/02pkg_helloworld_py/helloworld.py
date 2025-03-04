import rclpy

#方法一（不推荐）
# def main():
#     #初始化ros2
#     rclpy.init()
#     #创建节点
#     node = rclpy.create_node("helloworld_node_py")
#     #输出文本
#     node.get_logger().info("hello world")
#     #释放资源
#     rclpy.shutdown()

#方法二（推荐）
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("helloworld_node_py")
        self.get_logger().info("hello world!")

def main():
    #初始化ros2
    rclpy.init()
    #创建节点
    node = MyNode()
    #释放资源
    rclpy.shutdown()

if __name__ == '__main__':
    main()
