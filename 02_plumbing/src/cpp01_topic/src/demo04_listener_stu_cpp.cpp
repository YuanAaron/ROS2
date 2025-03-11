#include "rclcpp/rclcpp.hpp"
#include "base_interfaces/msg/student.hpp"

using base_interfaces::msg::Student;

class ListenerStu: public rclcpp::Node
{

public:
ListenerStu(): Node("listenerstu_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "订阅节点创建!");
        _subscriber = this->create_subscription<Student>("topic",10, std::bind(&ListenerStu::topic_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<Student>::SharedPtr _subscriber;
    void topic_callback(const Student& stu) const
    {
        RCLCPP_INFO(this->get_logger(), "订阅的消息：name = %s, age = %d, height = %.2f", stu.name.c_str(), stu.age, stu.height);
    } 
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ListenerStu>();
    rclcpp::spin(node);
    rclcpp::shutdown();
  return 0;
}