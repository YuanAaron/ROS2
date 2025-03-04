#include "rclcpp/rclcpp.hpp"

//方法一（不推荐）：直接实例化的Ｎode对象

/* int main(int argc, char ** argv)
{
	//初始化ros2
	rclcpp::init(argc, argv);
	//创建节点
	auto node = rclcpp::Node::make_shared("helloworld_node_cpp");
	//输出文本
	RCLCPP_INFO(node -> get_logger(), "hello world");
	//释放资源
	rclcpp::shutdown();
  	return 0;
} */

//方法二（推荐）：以继承Node的方式来创建节点对象（该方式可以在一个进程内组织多个节点）

class MyNode: public rclcpp::Node
{
public:
	MyNode():Node("helloworld_node_cpp")
	{
		RCLCPP_INFO(get_logger(), "hello world!");
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<MyNode>();
	rclcpp::shutdown();
	return 0;
}
