import rclpy
from rclpy.node import Node
from base_interfaces.msg import Student

class TalkerStu(Node):
    def __init__(self):
        super().__init__("talkerstu_node_py")
        self.get_logger().info("发布方创建了！")
        self._publisher = self.create_publisher(Student, "topic", 10)
        self._timer = self.create_timer(0.5, self.timer_callback)
        self._count = 0
    
    def timer_callback(self):
        stu = Student()
        stu.name = "lisi"
        stu.age = self._count
        stu.height = 1.75
        self._count += 1
        self._publisher.publish(stu)
        self.get_logger().info("发布的消息：name = %s, age = %d, height = %.2f" %(stu.name, stu.age, stu.height))

def main():
    rclpy.init()
    node = TalkerStu()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()