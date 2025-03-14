import rclpy
from rclpy.node import Node
from base_interfaces.srv import AddInts

class Server(Node):
    def __init__(self):
        super().__init__("server_node_py")
        self._server = self.create_service(AddInts, "add_ints", self.add_callback)
        self.get_logger().info("服务端启动！")

    def add_callback(self, req, res):
        res.sum = req.num1 + req.num2
        self.get_logger().info("请求数据：（%d, %d），响应结果：（%d）" %(req.num1, req.num2, res.sum))
        return res

def main():
    rclpy.init()
    node = Server()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()