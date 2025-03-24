## 遇到的问题
zsh下ros2和colcon无法自动补全

sudo vim /opt/ros/humble/setup.zsh，然后在末尾添加如下两行即可：
```
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"
```

## 项目创建、编译、运行

### C++版
1. 进入工作空间的src目录创建功能包：ros2 pkg create 01pkg_helloworld_cpp --build-type ament_cmake --dependencies rclcpp --node-name helloworld，其中ament_cmake默认
2. 在工作空间目录：colcon build 或 colcon build --packages-select 功能包1 功能包2
3. 在工作空间目录：
	+ . install/setup.bash 或 source install/setup.bash（否则在其他终端下无法运行）
	+ ros2 run 01pkg_helloworld_cpp helloworld

### Python版
1. 进入工作空间的src目录创建功能包：ros2 pkg create 02pkg_helloworld_py --build-type ament_python --dependencies rclpy --node-name helloworld
2. 在工作空间目录：colcon build 或 colcon build --packages-select 功能包1 功能包2
3. 在工作空间目录：
	+ . install/setup.bash 或 source install/setup.bash(同理）
	+ ros2 run 02pkg_helloworld_py helloworld
> 不建议：如果想省略每次执行都执行source install/setup.bash，可以将该对应功能包的该命令添加到.bashrc，即echo "source {工作空间路径}/install/setup.bash" >> ~/.bashrc。

### vscode

1. 使用vscode后C++代码报错（其实代码是没问题的，只是vscode找不到头文件而异）：检测到#include错误，请更新includePath。修正方法是：快速修复 -> 编辑includePath设置 -> 在包含路径中添加/opt/ros/humble/include/**即可。

## ros2体系框架
### [ROS2的文件系统](https://zhuanlan.zhihu.com/p/655747465)

不推荐前面的直接实例化的Node对象，推荐使用继承的方式来创建节点对象，因为继承方式可以在一个进程内组织多个节点

[在一个进程内组织多个节点](https://fishros.org/doc/ros2/humble/Tutorials/Intermediate/Composition.html)

> 代码中的init（初始化）和shutdown（资源释放）的作用：我们要知道构建的程序可能由若干步骤/阶段组成（初始化-> 创建节点对象->日志输出->数据的发布/订阅->...->资源释放），不同步骤之间使用Context对象（上下文，相当于一个容器，既可以存储数据，又可以从中读取数据）实现数据的传递/流转。因此，初始化其实就是要创建Context对象，资源释放就是要销毁Context对象。

ros2 pkg查询功能包相关信息：
+ ros2 pkg executables [包名]：列出所有或指定功能包下的可执行程序
+ ros2 pkg list：列出所有功能包
+ ros2 pkg prefix 包名：列出功能包路径
+ ros2 pkg xml 包名：输出功能包的package.xml内容

### ros2核心模块

#### 通信模块

#### 功能包应用

遇到的问题：zsh下需要使用apt search "ros-humble-*" | grep i turtlesim，bash下apt search ros-humble-* | grep i turtlesim就可以。

#### 分布式

#### 终端命令与rqt

#### launch文件

通过launch文件可以批量的启动多个ros2节点，这是构建大型项目时启动多节点的常用方式。

ros2 launch turtlesim hello.launch.py

#### TF坐标变换

#### 可视化

ros2内置了三维可视化工具rviz2

### ros2技术支持

[ROS包文档](https://docs.ros.org/)

[ROS问答](https://answers.ros.org/questions/)

[ROS论坛](https://discourse.ros.org/)

[ROS包索引](https://index.ros.org/)

[问题追踪器](https://github.com/ros2/ros2/issues)

### ros2应用方向

NAV2(导航)

OpenCV（计算机视觉）

MoveIt（机械臂）

Autoware（自动驾驶）

microROS

Open Robotics

PX4

ROS-Industrial

## ros2通信机制

### 节点
### 话题

### 通信模型
#### 话题通信

一种单向通信模型，发布方发布数据，订阅方订阅数据，数据流单向的由发布方传输到订阅方。一般应用于不断更新的、少逻辑处理的数据传输场景。

话题通信的发布方和订阅方是一种多对多的关系。

ros2 pkg create cpp01_topic --build-type ament_cmake --dependencies rclcpp std_msgs base_interfaces --node-name demo01_talker_str_cpp

ros2 pkg create py01_topic --build-type ament_python --dependencies rclpy std_msgs base_interfaces --node-name demo01_talker_str_py

验证发布方是否将话题数据发送出去：ros2 topic echo /topic，其中topic为话题名称。

这一节还涉及到自定义接口消息（文件名首字母必须大写），验证方式为：ros2 interface show base_interfaces/msg/Student

C++代码需要配置VSCode的c_cpp_properties.json文件，在includePath属性下添加一行："${workspaceFolder}/install/base_interfaces_demo/include/**"，添加完毕后，包含相关头文件时，就不会抛出异常了。

Python代码配置VSCode中settings.json文件，在python.autoComplete.extraPaths和python.analysis.extraPaths属性下添加一行："${workspaceFolder}/install/base_interfaces_demo/local/lib/python3.10/dist-packages"，添加完毕后，代码可以高亮显示且可以自动补齐.

rqt查看话题通信的计算图：Plugins -> Introspection -> Node Graph，这可以验证发布方和订阅方是一种多对多的关系。


#### 服务通信

一种基于请求响应的通信模型，在通信双方中，客户端发送请求数据到服务端，服务端响应结果给客户端。适用于偶然的、对实时性有要求、有一定逻辑处理需求的数据传输场景。

服务通信的服务端和客户端是一对多的关系。

验证服务端代码：ros2 service call /add_ints base_interfaces/srv/AddInts "{'num1': 10, 'num2': 20}"，其中add_ints为服务话题名称，base_interfaces/srv/AddInts为服务接口消息类型，最后是json格式的请求数据。

#### 动作通信

一种带有连续反馈的通信模型，在通信双方中，客户端发送请求数据到服务端，服务端响应结果给客户端，但是在服务端接收到请求到产生最终响应的过程中，会发送连续的反馈信息到客户端。适用于耗时的请求响应场景，用以获取连续的状态反馈。

1. 从结构角度看，动作通信由目标、反馈、结果三部分组成；
2. 就功能而言，动作通信类似于服务通信，客户端发送请求到服务端，并接收服务端响应的最终结果，但动作通信可以在请求响应过程中获取连续反馈，且也可以向服务端发送任务取消请求；
3. 就地层实现而言，动作通信是建立在话题通信和服务通信之上的，目标发送实现是对服务通信的封装，结果的获取也是对服务通信的封装，而连续反馈是对话题通信的封装。

验证服务端代码：ros2 action send_goal /action_sum base_interfaces/action/Progress -f "{'num': 10}"

python代码的技巧：当python代码没有代码提示时，该怎么办？
self.get_logger().info(str(type(goal_handle))) #或 goal_handle.__str__()，后者不如前者
方法一：从输出的结果<class 'rclpy.action.client.ClientGoalHandle'>去查看其源码，就可以知道该类有哪些属性和方法；
方法二：将输出结果<class 'rclpy.action.client.ClientGoalHandle'>导入进来，然后显式声明goal_handle的类型，即goal_handle:ClientGoalHandle = future.result()，这样后面使用goal_handle时就有属性和方法的提示了
        

#### 参数通信（基于服务通信）

一种基于共享的方式实现不同节点之间数据交互的通信模型，在通信双方中，参数服务端（保存参数的节点）可以设置数据，而参数客户端（调节参数的节点）可以连接参数服务端并操作服务端数据。

参数服务保存的数据类似于编程中“全局变量”的概念，可以在不同的节点之间共享数据（设置私有就不行了）。比如在一个节点下保存车辆尺寸数据，其他节点可以访问该节点并操作这些数据。

参数客户端与服务端的交互基于请求响应，且参数通信的实现本质是对服务通信的进一步封装。

查看参数服务端创建了哪些参数：ros2 param list
查看参数值：ros2 param get /param_server_node_cpp car_type

为什么参数客户端通过参数服务端的节点名称关联，而不像服务通信用话题名称关联？

原因：
1. 参数服务端启动后，其底层封装了多个服务通信的服务端（这个可以通过ros2 service list查看）;
2. 每个服务通信的服务端的话题都是采用 /参数服务端节点名称/xxx 的形式;
3. 参数客户端启动后，也会封装多个服务通信的客户端，这些客户端使用相同的话题与服务通信的服务端相关联，因此，参数客户端在创建时需要使用到参数服务端节点名称。

注意：ROS2的Python客户端暂时没有提供参数客户端专用的API，但是参数通信的底层是基于服务通信的，所以可以通过服务通信操作参数服务端的参数。

### 接口

msg文件是用于定义话题通信中数据载体的接口文件。

srv文件是用于定义服务通信中数据载体的接口文件。

action文件是用于定义动作通信中数据载体的接口文件。

通信机制小结：
+ 相同点：通信必然涉及到双方，双方需要通过“话题”关联，通信还必然涉及到数据（一般可以通过接口文件来定义数据格式，参数通信是通过参数类封装数据）；
+ 不同点：话题通信基于广播的单向数据交互模式；服务通信是基于请求响应的问答式数据交互模式；动作通信则是在请求响应的过程中又包含连续反馈的数据交互模式；参数服务是基于服务通信的，可以在不同节点间实现数据共享。