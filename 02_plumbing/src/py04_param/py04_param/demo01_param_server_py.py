import rclpy
from rclpy.node import Node

class ParamServer(Node):
    def __init__(self):
        super().__init__("param_server_node_py")
        self.get_logger().info("----------演示参数API的使用-----------")

        #参数对象创建
        p1 = rclpy.Parameter("car_name", value = "Tiger")
        p2 = rclpy.Parameter("width", value = 0.5)
        p3 = rclpy.Parameter("wheels", value = 4)

        #参数属性的解析
        #获取参数值
        self.get_logger().info("car_name= %s" %p1.value)
        self.get_logger().info("width= %.2f" %p2.value)
        self.get_logger().info("wheels= %d" %p3.value)

        #获取参数键
        self.get_logger().info("p1 name = %s" %p1.name)

def main():
    rclpy.init()
    node = ParamServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
