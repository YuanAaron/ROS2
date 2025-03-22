/*
    需求：编写参数客户端，获取或修改服务端参数(注意：不允许增删)。
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.查询参数；
            3-2.修改参数；
        4.创建节点对象指针，调用参数操作函数；
        5.释放资源。
*/

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ParamClient: public rclcpp::Node
{

public:
    ParamClient(): Node("param_client_node_cpp")
    {
        //参数查看rclcpp::SyncParametersClient的构造函数
        //注意：第二个参数可是服务端的节点名称哦
        //为什么参数客户端通过参数服务端的节点名称关联，而不像服务通信用话题名称关联？
        //原因：1、参数服务端启动后，其底层封装了多个服务通信的服务端（这个可以通过ros2 service list查看），
        //2、每个服务通信的服务端的话题都是采用 /参数服务端节点名称/xxx 的形式，
        //3、参数客户端启动后，也会封装多个服务通信的客户端，这些客户端使用相同的话题与服务通信的服务端相关联，
        //因此，参数客户端在创建时需要使用到参数服务端节点名称。
        _client = std::make_shared<rclcpp::SyncParametersClient>(this, "param_server_node_cpp");
    }

    bool connect_server()
    {
        while (!_client -> wait_for_service(1s)) 
        {
            if(!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "强制终止客户端！");
                return false;
            }
            RCLCPP_INFO(this -> get_logger(), "尝试连接服务端中，请稍后...");
        }

        return true;
    }

    //查询参数
    void get_param()
    {
        RCLCPP_INFO(this -> get_logger(), "-----------参数客户端查询参数------------");
        //根据键获取参数值
        double height = _client -> get_parameter<double>("height");
        RCLCPP_INFO(this->get_logger(),"height = %.2f", height);
        //判断是否包含键为xxx的参数
        RCLCPP_INFO(this->get_logger(),"car_type 存在吗？%d", _client -> has_parameter("car_type"));
        //根据键组成的vector获取一些参数对象
        auto params = _client -> get_parameters({"car_type","height","wheels"});
        for (auto &param : params)
        {
            RCLCPP_INFO(this->get_logger(),"%s = %s", param.get_name().c_str(), param.value_to_string().c_str());
        }
    }

    //修改参数
    void update_param()
    {
        RCLCPP_INFO(this->get_logger(),"-----------参数客户端修改参数-----------");
        _client -> set_parameters({
            rclcpp::Parameter("car_type","Mouse"),
            rclcpp::Parameter("height",2.0),
            //这是服务端不存在的参数，只有服务端设置了rclcpp::NodeOptions().allow_undeclared_parameters(true)时，
            // 这个参数才会被成功设置。
            rclcpp::Parameter("width",0.15),
            rclcpp::Parameter("wheels",6),
            rclcpp::Parameter("length",5.0) //set_parameters参数服务端不存在键的参数，前提：同样需要保证服务端设置了rclcpp::NodeOptions().allow_undeclared_parameters(true)
        });

        //查询写在这里仅仅为了测试length的增加
        RCLCPP_INFO(this->get_logger(),"length = %.2f", _client-> get_parameter<double>("length"));

    }

private:
    rclcpp::SyncParametersClient::SharedPtr _client;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParamClient>();
    bool flag = node -> connect_server();
    if(!flag)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "参数服务端连接失败！");
        return 1;
    }

    node -> get_param();
    node -> update_param();
    node -> get_param();

    rclcpp::shutdown();
    return 0;
}