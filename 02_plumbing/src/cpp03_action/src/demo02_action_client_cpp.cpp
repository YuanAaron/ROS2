/*
    需求：编写动作客户端实现，可以提交一个整型数据到服务端，并处理服务端的连续反馈以及最终返回结果。
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
        3-1.创建动作客户端；
        3-2.发送目标请求；
        3-3.处理目标发送后的响应；
        3-4.处理连续反馈；
        3-5.处理最终响应。
        4.调用spin函数，并传入节点对象指针；
        5.释放资源。
*/

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces/action/progress.hpp"

using base_interfaces::action::Progress;
using GoalHandleProgress = rclcpp_action::ClientGoalHandle<Progress>;
using std::placeholders::_1;
using std::placeholders::_2;

class ActionClient: public rclcpp::Node
{

public:
    ActionClient(): Node("actionclient_node_cpp")
    {
        //创建动作客户端
        _action_client = rclcpp_action::create_client<Progress>(this, "action_sum");
    }

    //bug：当使用命令行 ros2 action send_goal /action_sum base_interfaces/action/Progress -f "{'num': 20}"作为动作客户端，
    //按下ctrl+c后，动作服务端停止响应；但当使用自己写的动作客户端程序，按下ctrl+c后，动作服务端并没有停止响应，而是一直响应到计算结束。
    //原因：自己写的动作客户端，按下ctrl+c只是结束了动作客户端程序，并不会发送取消任务请求；

    //TODO: fix bug：在析构函数中发送取消任务请求，这样在计算过程中，按下ctrl+c后，动作服务端会停止响应；但副作用是
    //在计算完成按ctrl+c正常关闭的情况下，也会发送取消任务的请求（只不过此时动作服务端没反应）。
    ~ActionClient()
    {
        RCLCPP_INFO(this->get_logger(), "Canceling Goal");
        _action_client -> async_cancel_all_goals();
    }

    //发送目标请求
    void send_goal(int64_t num)
    {
        if(!_action_client)
        {
            RCLCPP_ERROR(this->get_logger(), "动作客户端尚未被初始化");
            return;
        }

        if(!_action_client -> wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "连接动作服务端失败!");
            return;
        }

        auto goal = Progress::Goal();
        goal.num = num;
        RCLCPP_INFO(this->get_logger(), "发送请求数据!");

        rclcpp_action::Client<Progress>::SendGoalOptions send_goal_options;
        send_goal_options.goal_response_callback = std::bind(&ActionClient::goal_response_callback,this, _1);
        send_goal_options.feedback_callback = std::bind(&ActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&ActionClient::result_callback, this, _1);
        //关键代码
        _action_client -> async_send_goal(goal, send_goal_options);
    }

private:
    rclcpp_action::Client<Progress>::SharedPtr _action_client;
    
    //处理目标发送后的响应
    //（该方法的写法可以　通过SendGoalOptions的源码找到
    //     using GoalResponseCallback = std::function<void (typename GoalHandle::SharedPtr)>;
    //来获得）
    void goal_response_callback(GoalHandleProgress::SharedPtr goal_handle)
    {
        //如果服务端拒绝，这里的goal_handle为nullptr
        if(!goal_handle)
        {
            RCLCPP_ERROR(this -> get_logger(), "目标请求被服务器拒绝!");
        }
        else{
            RCLCPP_INFO(this->get_logger(), "目标被接收，等待结果中...");
        }
    }

    //处理连续反馈
    //（该方法的写法可以　通过SendGoalOptions的源码找到
    //     using FeedbackCallback = std::function<void (typename ClientGoalHandle<ActionT>::SharedPtr, const std::shared_ptr<const Feedback>)>;
    //来获得）
    void feedback_callback(GoalHandleProgress::SharedPtr, const std::shared_ptr<const Progress::Feedback> feedback)
    {
        int32_t progress = feedback -> progress * 100;
        RCLCPP_INFO(this->get_logger(), "当前进度：%d%%", progress);
    }

    //处理最终响应
    //（该方法的写法可以　通过SendGoalOptions的源码找到
    //     using ResultCallback = std::function<void (const WrappedResult & result)>;
    //来获得）
    void result_callback(const GoalHandleProgress::WrappedResult& result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED: //TODO: 下面三种情况怎么才会出现呢？？？
            RCLCPP_ERROR(this->get_logger(), "任务被中止");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "任务被取消");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "未知异常");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "任务执行完毕，最终结果：%ld", result.result ->sum);
    }
};


int main(int argc, char** argv)
{
    if(argc != 2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "请提交一个整数！");
        return 1;
    }

    rclcpp::init(argc, argv);
    auto action_client = std::make_shared<ActionClient>();
    action_client -> send_goal(atoi(argv[1]));
    rclcpp::spin(action_client);
    rclcpp::shutdown();
    return 0;
}