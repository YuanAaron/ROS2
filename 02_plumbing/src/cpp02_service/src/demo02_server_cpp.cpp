/*  
  需求：编写服务端，接收客户端发送请求，提取其中两个整型数据，相加后将结果响应回客户端。
  步骤：
    1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
      3-1.创建服务端；
      3-2.回调函数中处理请求并响应结果。
    4.调用spin函数，并传入节点对象指针；
    5.释放资源。
*/

#include "rclcpp/rclcpp.hpp"
#include "base_interfaces/srv/add_ints.hpp"

using base_interfaces::srv::AddInts;

class Server: public rclcpp::Node
{

public:
    Server(): Node("server_node_cpp")
    {
        //创建服务端
        _server = this -> create_service<AddInts>("add_ints", std::bind(&Server::add, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this -> get_logger(), "服务端启动完毕，等待请求提交...");
    }

private:
    rclcpp::Service<AddInts>::SharedPtr _server;
    //处理请求并响应结果
    void add(const AddInts::Request::SharedPtr req, const AddInts::Response::SharedPtr res)
    {
        res -> sum = req -> num1 + req -> num2;
        RCLCPP_INFO(this -> get_logger(), "请求数据：（%d, %d），响应结果为：%d", req -> num1, req -> num2, res -> sum); 
    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Server>();
    rclcpp::spin(node);
    rclcpp::shutdown();
  return 0;
}