import sys
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from base_interfaces.srv import AddInts

class Client(Node):
    def __init__(self):
        super().__init__("client_node_py")
        self._client = self.create_client(AddInts, "add_ints")
        self.get_logger().info("客户端创建，等待连接服务端...")

        #经过验证，python实现中不存在cpp实现中的ctrl+c问题
        while not self._client.wait_for_service(1.0):
            self.get_logger().info("尝试连接服务端中，请稍后...")

    def send_request(self):
        req = AddInts.Request()
        req.num1 = int(sys.argv[1])
        req.num2 = int(sys.argv[2])
        return self._client.call_async(req)

def main():
    if len(sys.argv) != 3:
        get_logger("rclpy").error("请提交两个整数！")
        return
      
    rclpy.init()
    client = Client()

    future = client.send_request()
    rclpy.spin_until_future_complete(client, future)
    #通过异常来判断是否请求成功
    try:
        res = future.result()
        client.get_logger().info("请求正常处理，响应结果为：%d" %res.sum)
    except Exception as e:
        client.get_logger().error("请求失败：%r" %e)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
