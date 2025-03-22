/*
    需求：编写参数服务端，设置并操作参数。
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.增加参数；
            3-2.查询参数；
            3-3.修改参数；
            3-4.删除参数。
        4.创建节点对象指针，调用参数操作函数，并传递给spin函数；
        5.释放资源。

    参数由键、值和描述符三部分组成：
    键是字符串类型，
    值可以是bool、int64、float64、string、byte[]、bool[]、int64[]、string[]中的任一类型，
    描述符默认情况下为空，但可以设置参数描述、参数数据类型、取值范围或其他约束等信息。
*/

#include "rclcpp/rclcpp.hpp"

class ParamServer: public rclcpp::Node
{

public:
    ParamServer(): Node("param_server_node_cpp", rclcpp::NodeOptions().allow_undeclared_parameters(true)) //允许参数服务端做删除操作，则需要此设置
    {
        // RCLCPP_INFO(this -> get_logger(), "-----------演示参数API的使用------------");
        // //参数对象的创建
        // rclcpp::Parameter p1("car_name", "Tiger"); //参数值为字符串类型
        // rclcpp::Parameter p2("width", 0.15);
        // rclcpp::Parameter p3("wheels", 2);

        // //参数属性解析
        // //获取参数值并转换成相应的数据类型
        // RCLCPP_INFO(this -> get_logger(), "car_name = %s", p1.as_string().c_str());
        // RCLCPP_INFO(this -> get_logger(), "width = %.2f", p2.as_double());
        // RCLCPP_INFO(this -> get_logger(), "wheels = %ld", p3.as_int());

        // //获取参数的键
        // RCLCPP_INFO(this -> get_logger(), "p1 name = %s", p1.get_name().c_str());
        // //获取参数值类型
        // RCLCPP_INFO(this->get_logger(), "p2 type_name = %s", p2.get_type_name().c_str());
        // //将参数值转换成字符串类型
        // RCLCPP_INFO(this -> get_logger(), "p3 value_to_string = %s", p3.value_to_string().c_str());

        // //还可以去研究一下rclcpp::ParameterValue
    }

    //增加参数
    void declare_param()
    {
        //声明参数并设置默认值
        this -> declare_parameter("car_type", "Tiger");
        this -> declare_parameter("height", 1.50);
        this -> declare_parameter("wheels", 4);

        //this -> set_parameter除了可以修改参数已有键对应的值外，还可以新增参数。
        //但前提也像删除一样需要设置：rclcpp::NodeOptions().allow_undeclared_parameters(true)
        this -> set_parameter(rclcpp::Parameter("undcl_test", 100));
    }

    //查询参数
    void get_param()
    {
        RCLCPP_INFO(this -> get_logger(), "-------------参数服务端查询参数------------");
        //根据键获取参数对象
        rclcpp::Parameter p = this -> get_parameter("car_type");
        RCLCPP_INFO(this -> get_logger(), "car_type: %s", p.as_string().c_str());
        RCLCPP_INFO(this -> get_logger(), "height: %.2f", this -> get_parameter("height").as_double());
        RCLCPP_INFO(this -> get_logger(), "wheels: %ld", this -> get_parameter("wheels").as_int());
        RCLCPP_INFO(this -> get_logger(), "undcl_test: %ld", this -> get_parameter("undcl_test").as_int());     

        //判断是否包含键为xxx的参数
        RCLCPP_INFO(this -> get_logger(), "包含car_type？: %d", this -> has_parameter("car_type"));
        RCLCPP_INFO(this -> get_logger(), "包含car_typexxxx: %d", this -> has_parameter("car_typexxxx"));

        //根据键组成的vector获取一些参数对象
        auto params = this -> get_parameters({"car_type", "height", "wheels"});
        for(auto& param : params)
        {
            RCLCPP_INFO(this -> get_logger(), "name = %s, value = %s, type = %s", 
                        param.get_name().c_str(), param.value_to_string().c_str(), param.get_type_name().c_str());
        }

    }

    //修改参数
    void update_param()
    {
        RCLCPP_INFO(this -> get_logger(), "---------参数服务端修改参数------------");
        this -> set_parameter(rclcpp::Parameter("height", 1.75));
        RCLCPP_INFO(this -> get_logger(), "height: %.2f", this -> get_parameter("height").as_double()); 
    }

    //删除参数
    void delete_param()
    {
        RCLCPP_INFO(this -> get_logger(), "---------参数服务端删除参数--------------");
        //this->undeclare_parameter("car_type"); //不能删除声明的参数（会抛出异常），即通过declare_parameter增加的参数
        //RCLCPP_INFO(this->get_logger(),"删除操作后，car_type还存在吗? %d", this->has_parameter("car_type"));
        RCLCPP_INFO(this->get_logger(),"删除操作前，undcl_test存在吗? %d", this->has_parameter("undcl_test"));
        this->undeclare_parameter("undcl_test"); //可以删除通过set_parameter增加的参数
        RCLCPP_INFO(this->get_logger(),"删除操作后，undcl_test存在吗? %d", this->has_parameter("undcl_test"));
    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParamServer>();
    node -> declare_param();
    node -> get_param();
    node -> update_param();
    node -> delete_param();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
