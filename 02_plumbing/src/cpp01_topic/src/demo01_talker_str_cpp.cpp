/*  
  需求：以某个固定频率发送文本“hello world!”，文本后缀编号，每发送一条消息，编号递增1。
  步骤：
    1.包含头文件；
    2.初始化 ROS2 客户端； 
    3.定义节点类；
      3-1.创建发布方；
      3-2.创建定时器；
      3-3.组织消息并发布。
    4.调用spin函数，并传入节点对象指针；
    5.释放资源。
*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

//using namespace std::chrono_literals;

class Talker: public rclcpp::Node
{

public:
  Talker(): Node("talker_node_cpp"),_count(0)
  {
    RCLCPP_INFO(this->get_logger(), "发布节点创建!");
    //创建发布方
    _publisher = this->create_publisher<std_msgs::msg::String>("topic",10); //发布的话题名称为topic，队列长度为10
    //创建定时器
    //可调用对象可以是成员函数绑定std::bind、lambda表达式等
    //_timer = this->create_wall_timer(500ms, std::bind(&Talker::timer_callback, this));//把 Talker 类的 timer_callback 成员函数绑定到当前对象 this 上，生成一个可调用对象。
    _timer = this->create_wall_timer(std::chrono::milliseconds(500), [this](){
      //组织消息并发布
      auto message = std_msgs::msg::String();
      message.data = "hello world! " + std::to_string(_count++);
      RCLCPP_INFO(this->get_logger(), "发布的消息为：'%s'", message.data.c_str());
      _publisher->publish(message);
    });
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
  rclcpp::TimerBase::SharedPtr _timer;
  size_t _count;

  void timer_callback()
  {
    //组织消息并发布
    auto message = std_msgs::msg::String();
    message.data = "hello world! " + std::to_string(_count++);
    RCLCPP_INFO(this->get_logger(), "发布的消息：'%s'", message.data.c_str());
    _publisher->publish(message);
  }
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Talker>();
  //不断地循环检查被其运行的节点是否收到新的话题数据等事件，直到该节点被关闭为止
  //如果这里没有spin函数，回调函数timer_callback不会被执行
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
