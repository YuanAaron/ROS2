"""
    ROS2的Python客户端暂时没有提供参数客户端专用的API，
    但是参数通信的底层是基于服务通信的，所以可以通过服务通信操作参数服务端的参数。
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import ListParameters
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterValue
from ros2param.api import get_parameter_value

class ParamClient(Node):
    def __init__(self):
        super().__init__("param_client_node_py")
    
    def list_params(self):
        self._client = self.create_client(ListParameters, "/param_server_node_py/list_parameters")
        self.get_logger().info("客户端创建，等待连接服务端...")

        while not self._client.wait_for_service(1.0):
            self.get_logger().info("列出参数尝试连接服务端中，请稍后...")
        
        req = ListParameters.Request()
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def get_params(self, names):
        self._client = self.create_client(GetParameters, "/param_server_node_py/get_parameters")
        self.get_logger().info("客户端创建，等待连接服务端...")

        while not self._client.wait_for_service(1.0):
            self.get_logger().info("根据键列表获取参数尝试连接服务端中，请稍后...")
        
        req = GetParameters.Request()
        #@property装饰器把names方法转化为一个只读属性，这样就可以像访问剖同属性一样访问names，而不是调用方法；
        #@name.setter装饰器定义了一个setter方法，当你尝试为names属性赋值时，就会调用该方法
        req.names = names
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def update_params(self):
        self._client = self.create_client(SetParameters, "/param_server_node_py/set_parameters")
        self.get_logger().info("客户端创建，等待连接服务端...")

        while not self._client.wait_for_service(1.0):
            self.get_logger().info("修改参数尝试连接服务端中，请稍后...")

        req = SetParameters.Request()

        p1 = Parameter()
        p1.name = "car_type"

        # v1 = ParameterValue()
        # v1.type = ParameterType.PARAMETER_STRING
        # v1.string_value = "Pig"
        # p1.value = v1
        p1.value = get_parameter_value(string_value="Pig")

        p2 = Parameter()
        p2.name = "height"
         
        v2 = ParameterValue()
        v2.type = ParameterType.PARAMETER_DOUBLE
        v2.double_value = 0.3
        p2.value = v2
        #p2.value = get_parameter_value(string_value="0.3")

        req.parameters = [p1, p2]
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    client = ParamClient()

    #获取参数列表
    client.get_logger().info("------获取参数列表------")
    response = client.list_params()
    for name in response.result.names:
        client.get_logger().info(name)

    ##根据键组成的列表获取一些参数
    client.get_logger().info("------根据键组成的列表获取一些参数-----")
    names = ["height", "car_type"]
    response = client.get_params(names)
    #print(response.values)
    for v in response.values:
        if v.type == ParameterType.PARAMETER_STRING:
            client.get_logger().info("字符串值:%s" % v.string_value)
        elif v.type == ParameterType.PARAMETER_DOUBLE:
            client.get_logger().info("浮点值:%.2f" % v.double_value)

    #设置参数
    client.get_logger().info("------设置参数--------")
    response = client.update_params()
    #print(response.results)
    results = response.results
    client.get_logger().info("设置了 %d 个参数" %len(results))
    for result in results:
        if not result.successful:
            client.get_logger().info("参数设置失败")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()