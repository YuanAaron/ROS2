import rclpy
from rclpy.node import Node

class ParamServer(Node):
    def __init__(self):
        super().__init__("param_server_node_py", allow_undeclared_parameters = True)

        # self.get_logger().info("----------演示参数API的使用-----------")

        # #参数对象创建
        # p1 = rclpy.Parameter("car_name", value = "Tiger")
        # p2 = rclpy.Parameter("width", value = 0.5)
        # p3 = rclpy.Parameter("wheels", value = 4)

        # #参数属性的解析
        # #获取参数值
        # self.get_logger().info("car_name= %s" %p1.value)
        # self.get_logger().info("width= %.2f" %p2.value)
        # self.get_logger().info("wheels= %d" %p3.value)

        # #获取参数键
        # self.get_logger().info("p1 name = %s" %p1.name)

    #增加参数
    def declare_param(self):
        self.declare_parameter("car_type", "Tiger")
        self.declare_parameter("height", 1.5)
        self.declare_parameter("wheels", 4)
        #同样需要提前设置allow_undeclared_parameters = True
        self.set_parameters([rclpy.Parameter("car_type", value = "Mouse"), rclpy.Parameter("undcl_test", value = 100)])

    #查询参数
    def get_param(self):
        self.get_logger().info("-------参数服务端查询参数-------")
        #根据键获取单个参数对象
        p = self.get_parameter("car_type")
        self.get_logger().info("%s = %s" %(p.name, p.value))

        #判断是否包含键为xxx的参数
        self.get_logger().info("包含car_type？：%d" %self.has_parameter("car_type"))
        self.get_logger().info("包含width？：%d" %self.has_parameter("width"))

        #根据键组成的列表获取一些参数对象
        params = self.get_parameters(["car_type", "height", "wheels"])
        for param in params:
            self.get_logger().info("%s ---> %s" %(param.name, param.value))

    #修改参数
    def update_param(self):
        self.get_logger().info("-------参数服务端修改参数---------")
        self.set_parameters([rclpy.Parameter("car_type", value = "horse")])
        self.get_logger().info("修改后：car_type = %s" %self.get_parameter("car_type").value)

    #删除参数
    def delete_param(self):
        self.get_logger().info("-------参数服务端删除参数----------")
        #self.undeclare_parameter("height") #python实现竟然可以删除声明的参数，且不需要设置allow_undeclared_parameters = True
        #self.get_logger().info("删除后包含 height吗？：%d" %self.has_parameter("height"))
        self.get_logger().info("删除前包含 car_type吗？：%d" %self.has_parameter("car_type"))
        self.undeclare_parameter("car_type") 
        self.get_logger().info("删除后包含 car_type吗？：%d" %self.has_parameter("car_type"))

def main():
    rclpy.init()
    node = ParamServer()
    node.declare_param()
    node.get_param()
    node.update_param()
    node.delete_param()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
