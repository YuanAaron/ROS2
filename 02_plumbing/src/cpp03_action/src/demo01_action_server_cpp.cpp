/*
  需求：编写动作服务端实现，可以提取客户端请求提交的整型数据，并累加从1到该数据之间的所有整数以求和，
       每累加一次都计算当前运算进度并连续反馈回客户端，最后再将求和结果返回给客户端。
  步骤：
    1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
      3-1.创建动作服务端；
      3-2.处理提交的目标值；
      3-４.处理客户端发送的取消请求；
      3-3.生成连续反馈和响应最终结果。
    4.调用spin函数，并传入节点对象指针；
    5.释放资源。

    //下面代码中typename的使用：
    //模板声明部分的typename：用于引入模板类型参数
    //其他的typename：主要用于解决依赖于模板参数的 嵌套名称 的歧义问题，告诉编译器这些名称代表的是类型而不是其他实体（如静态成员变量）。
                    比如以typename Server<ActionT>::SharedPtr为例，Server<ActionT>是一个依赖于模板参数ActionT的类模板实例，
                    而SharedPtr是Server<ActionT>类内部定义的一个类型别名。由于Server<ActionT>是依赖于模板参数ActionT的，编译器
                    在解析时无法确定SharedPtr是一个类型还是一个静态成员变量，所以需要使用typename来明确告知编译器是一个类型。
    template<typename ActionT, typename NodeT>
    typename Server<ActionT>::SharedPtr
    create_server(
      NodeT node,
      const std::string & name,
      typename Server<ActionT>::GoalCallback handle_goal,
      typename Server<ActionT>::CancelCallback handle_cancel,
      typename Server<ActionT>::AcceptedCallback handle_accepted,
      ...
      ){}

    //处理提交的目标值这一回调函数怎么定义出来的？
    //查看源代码：create_server -> Server-> using GoalCallback = std::function<GoalResponse(const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)>;
    //std::function是C++标准库中的一个通用的多态函数包装器，它可以存储、复制和调用任何可调用对象（函数、函数指针、成员函数指针、lambda表达式等）。
    //这里定义的std::function类型表示一个特定签名（即函数的返回值和参数列表）的函数。
*/

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces/action/progress.hpp"

using base_interfaces::action::Progress;
using GoalHandleProgress = rclcpp_action::ServerGoalHandle<Progress>;

class ActionServer: public rclcpp::Node
{

public:
    ActionServer(): Node("actionserver_node_cpp")
    {
      //创建动作服务端
      _action_server = rclcpp_action::create_server<Progress>(this, "action_sum", 
        std::bind(&ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1));
        RCLCPP_INFO(this -> get_logger(), "动作服务端创建，等待请求...");
    }

private:
    rclcpp_action::Server<Progress>::SharedPtr _action_server;

    //处理提交的目标值
    //using GoalCallback = std::function<GoalResponse(const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)>;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Progress::Goal> goal)
    {
      (void)uuid;
      RCLCPP_INFO(this->get_logger(), "接收到动作客户端请求，请求数字为：%ld", goal->num);
      if (goal->num < 1)
      {
        RCLCPP_ERROR(this -> get_logger(), "提交的目标值必须大于等于1");
        return rclcpp_action::GoalResponse::REJECT;
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    //处理客户端发送的取消请求
    //using CancelCallback = std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)>;
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleProgress> goal_handle)
    {
      (void)goal_handle;
      //当前是无条件同意取消，但在真实场景中需要判断是否允许取消请求
      RCLCPP_INFO(this -> get_logger(), "接收到任务取消请求");
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    //生成连续反馈和响应最终结果
    //using AcceptedCallback = std::function<void (std::shared_ptr<ServerGoalHandle<ActionT>>)>;
    void handle_accepted(const std::shared_ptr<GoalHandleProgress> goal_handle)
    {
      (void)goal_handle;
      //线程分离，主线程可以继续执行其他任务
      std::thread(std::bind(&ActionServer::execute, this, std::placeholders::_1), goal_handle).detach();
      //向线程函数传递参数goal_handle
      //std::thread t(execute, goal_handle); //错误：execute如果是全局函数可以这样调用
      //std::thread(&ActionServer::execute, this, goal_handle).detach(); //如果是成员函数，需要传递成员函数的指针、对象指针以及成员函数所需的参数给std::thread的构造函数
      //lambda表达式
      // std::thread(
      //   [this, goal_handle](){
      //     this -> execute(goal_handle);
      //   }
      // ).detach();
    }

    //耗时操作
    void execute(const std::shared_ptr<GoalHandleProgress> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "开始执行任务...");

      const int64_t num = goal_handle -> get_goal() -> num;
      auto feedback = std::make_shared<Progress::Feedback>();
      auto result = std::make_shared<Progress::Result>();

      rclcpp::Rate loop_rate(1.0); //设置休眠，1.0表示1HZ，即每秒执行一次；10.0表示10HZ，即1s执行10次
      int64_t sum = 0;
      for (int i = 1; (i <= num) && rclcpp::ok(); i++)
      {
        sum += i;

        //检查是否有取消请求（当客户端发送了取消请求，且服务端同意后，这里就不能继续执行下去了）
        if(goal_handle -> is_canceling())
        {
          result -> sum = sum;
          goal_handle -> canceled(result);
          RCLCPP_INFO(this->get_logger(), "任务取消!");
          return;
        }

        //生成连续反馈，返回给客户端
        feedback ->progress = (double_t)i / num;
        //关键代码1：参数为std::shared_ptr<base_interfaces::action::Progress_Feedback
        goal_handle -> publish_feedback(feedback); //使用话题通信发送连续反馈
        RCLCPP_INFO(this->get_logger(), "连续反馈中，进度为：%.2f", feedback -> progress);

        loop_rate.sleep();
      }

      //响应最终结果
      if(rclcpp::ok())
      {
        result->sum = sum;
        //关键代码2：参数为std::shared_ptr<base_interfaces::action::Progress_Result
        goal_handle -> succeed(result);
        RCLCPP_INFO(this->get_logger(), "任务完成!");
      }
    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
