import rclpy
from rclpy.node import Node
from base_interfaces.msg import Student

class ListenerStu(Node):
    def __init__(self):
        super().__init__("listenerstu_node_py")
        self.get_logger().info("订阅方创建了！")
        self._subscriber = self.create_subscription(Student, "topic", self.topic_callback, 10)

    def topic_callback(self, stu):
        self.get_logger().info("订阅到的消息：name = %s, age = %d, height = %.2f" %(stu.name, stu.age, stu.height))

def main():
    rclpy.init()
    node = ListenerStu()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()