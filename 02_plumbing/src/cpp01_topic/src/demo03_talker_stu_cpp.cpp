/*
    需求：以某个固定频率发送文本学生信息，包含学生的姓名、年龄、身高等数据。
*/

#include "rclcpp/rclcpp.hpp"
#include "base_interfaces/msg/student.hpp" //#include 指令是为了让编译器能够识别base_interfaces::msg::Student类型，而 using namespace 语句是为了简化类型的使用

using namespace std::chrono_literals; //或 using std::chrono_literals::operator""ms;可以将500ms（ms为字面量）转换为std::chrono::milliseconds(500)
using namespace base_interfaces::msg; //或 using base_interfaces::msg::Student;

class TalkerStu: public rclcpp::Node
{

public:
    TalkerStu(): Node("talkerstu_node_cpp"), _count(0)
    {
        RCLCPP_INFO(this->get_logger(), "发布节点创建!");
        _publisher = this->create_publisher<Student>("topic",10);
        _timer = this->create_wall_timer(500ms, std::bind(&TalkerStu::timer_callback, this));
    }

private:
    rclcpp::Publisher<Student>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;
    size_t _count;

    void timer_callback()
    {
        //组织并发布学生信息
        auto stu = Student();
        stu.name = "zhangsan";
        stu.age = _count++;
        stu.height = 1.70;
        RCLCPP_INFO(this->get_logger(), "发布的消息：name = %s, age = %d, height = %.2f", stu.name.c_str(), stu.age, stu.height);
        _publisher->publish(stu);
    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TalkerStu>();
    rclcpp::spin(node);
    rclcpp::shutdown();
  return 0;
}