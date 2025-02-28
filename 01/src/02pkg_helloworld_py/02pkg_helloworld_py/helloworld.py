import rclpy

def main():
    #初始化ros2
    rclpy.init()
    #创建节点
    node = rclpy.create_node("helloworld_node_py")
    #输出文本
    node.get_logger().info("hello world")
    #释放资源
    rclpy.shutdown()

if __name__ == '__main__':
    main()
