/*  
    需求：订阅发布方发布的消息，并输出到终端。
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.创建订阅方；
            3-2.处理订阅到的消息。
        4.调用spin函数，并传入节点对象指针；
        5.释放资源。
*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Listener: public rclcpp::Node
{
public:
  Listener(): Node("listener_node_cpp")
  {
    RCLCPP_INFO(this->get_logger(), "订阅节点创建!");
    //创建订阅方
    //std::bind用于将成员函数 Listener::topic_callback 绑定到 Listener 类的对象上，以便在订阅到消息时调用该回调函数。
    //std::placeholders::_1是一个占位符，表示当调用绑定的可调用对象时，第一个参数将被传递给 topic_callback 函数。（例子见后面）
    // _subscriber = this->create_subscription<std_msgs::msg::String>("topic",10, std::bind(&Listener::topic_callback, this, std::placeholders::_1));
    _subscriber = this->create_subscription<std_msgs::msg::String>("topic", 10, [this](const std_msgs::msg::String &msg){
      RCLCPP_INFO(this->get_logger(), "订阅的消息为：'%s'", msg.data.c_str());
    });
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscriber;
  void topic_callback(const std_msgs::msg::String& msg) const
  {
    RCLCPP_INFO(this->get_logger(), "订阅的消息：'%s'", msg.data.c_str());
  } 
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Listener>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

/* #include <iostream>
#include <functional>

int add(int a, int b) {
    return a + b;
}

int main() {
    auto add_5 = std::bind(add, std::placeholders::_1, 5);
    std::cout << add_5(3) << std::endl; // 输出 8
    return 0;
} */