/*  
  需求：编写客户端，发送两个整型变量作为请求数据，并处理响应结果。
  步骤：
    1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
      3-1.创建客户端；
      3-2.等待客户端连接到服务端；
      3-3.组织请求数据并发送；
    4.创建对象指针调用其功能,并处理响应；
    5.释放资源。
*/

#include "rclcpp/rclcpp.hpp"
#include "base_interfaces/srv/add_ints.hpp"

using base_interfaces::srv::AddInts;
using namespace std::chrono_literals;

class Client: public rclcpp::Node
{

public:
    Client(): Node("client_node_cpp")
    {
      //创建客户端
      _client = this -> create_client<AddInts>("add_ints");
      RCLCPP_INFO(this -> get_logger(), "客户端创建，等待连接服务端...");
    }

    //等待客户端连接到服务端
    bool connect_server()
    {
      //wait_for_service：在指定超时时间内连接服务器，如果成功返回true，否则返回false。
      //使用while循环后，会一直尝试连接，直到连接成功为止。
      while (!_client -> wait_for_service(1s)) 
      {
        //如果没有if代码块，就会存在一个bug：如果服务端不启动，正常情况下，每隔1s打印一次“尝试连接...”，
        //但当按下ctrl+c后程序似乎变得没有时间间隔了，疯狂输出“尝试连接...”，只有按下ctrl+z后才能退出。

        //为什么会程序会变得没有时间间隔了？（以下解释来自豆包）
        // 1. 在 Linux 等系统中，按下 Ctrl + C 会向当前运行的进程发送 SIGINT 信号。rclcp会捕获这个信号并触发相应的处理流程，通常会调用 rclcpp::shutdown() 函数来停止ROS2节点的运行。
        // 2. rclcpp::shutdown()的影响：
        //       节点状态变更：rclcpp::shutdown() 会改变节点的状态，使得 rclcpp::ok() 返回false。这表明ROS2节点已经进入关闭流程，许多 ROS2的内部机制会开始进行清理工作；
        //       资源释放与线程停止：该函数会释放ROS2节点占用的各种资源，包括线程、定时器、网络连接等。在释放资源的过程中，一些与超时机制相关的线程和定时器可能会被提前终止或失效。
        // 3. wait_for_service(1s)的超时机制失效：该函数依赖于 ROS2 内部的定时器和线程来实现 1s 的超时等待。当 rclcpp::shutdown() 被调用后，定时器和相关线程可能已经被清理或停止工作，
        //       导致该函数无法正常进行超时等待，立即返回 false，while 循环就会快速地不断执行，从而导致 “尝试连接...” 信息被疯狂输出。

        if(!rclcpp::ok()) //该函数在该客户端正常正常执行时返回true，当非正常执行时（比如按下了ctrl+c）返回false。
        {
          //如果这里使用this -> get_logger()，当按下ctrl+c时，可能会有bug：偶然性地抛出context相关的异常。
          //原因：按下ctrl+c表示结束ros2程序，释放资源（比如关闭context），而this->get_logger() 返回的日志记录器是与特定节点相关联的，节点又是在 rclcpp::Context 的管理下
          //创建和运行的。当 rclcpp::Context 被关闭（例如调用 rclcpp::shutdown）时，节点会被销毁，与之关联的日志记录器也将不再可用。
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "强制终止客户端！");
          return false;
        }
        RCLCPP_INFO(this -> get_logger(), "尝试连接服务端中，请稍后...");
      }
      return true;
    }

    //组织请求数据并发送
    //返回值：服务端的返回结果
    rclcpp::Client<AddInts>::FutureAndRequestId send_request(int32_t num1, int32_t num2)
    {
      auto req = std::make_shared<AddInts::Request>();
      req -> num1 = num1;
      req -> num2 = num2;
      return _client -> async_send_request(req);
    }

private:
    rclcpp::Client<AddInts>::SharedPtr _client;
};


int main(int argc, char** argv)
{
  if(argc != 3)
  {
    //相比于this->get_logger()，rclcpp::get_logger("name")创建logger对象不依赖于context。
    //这里无法使用this->get_logger()，因为还未init分配context
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "请提交两个整数！");
    return 1;
  }

  rclcpp::init(argc, argv);

  auto client = std::make_shared<Client>();
  bool flag = client -> connect_server();
  if(!flag)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "服务端连接失败！");
    return 1;
  }

  auto res = client -> send_request(atoi(argv[1]), atoi(argv[2]));
  //处理响应结果
  if(rclcpp::spin_until_future_complete(client, res) == rclcpp::FutureReturnCode::SUCCESS) //除了SUCCESS，还有INTERRUPTED和TIMEOUT
  {
    RCLCPP_INFO(client -> get_logger(), "请求正常处理，响应结果为：%d", res.get() -> sum);
  } else {
    RCLCPP_INFO(client -> get_logger(), "请求异常！");
  }

  rclcpp::shutdown();
  return 0;
}
