"""  
    需求：订阅发布方发布的消息，并输出到终端。
    步骤：
        1.导包；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.创建订阅方；
            3-2.处理订阅到的消息。
        4.调用spin函数，并传入节点对象；
        5.释放资源。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class listener(Node):
    def __init__(self):
        super().__init__("listener_node_py")
        self.get_logger().info("订阅方创建了！")
        self._subscriber = self.create_subscription(String, "topic", self.topic_callback, 10)

    def topic_callback(self, msg):
        self.get_logger().info("订阅到的消息：'%s'" %msg.data)

def main():
    rclpy.init()
    node = listener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()