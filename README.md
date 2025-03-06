## 遇到的问题
zsh下ros2和colcon无法自动补全

sudo vim /opt/ros/humble/setup.zsh，然后在末尾添加如下两行即可：
‵‵‵
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

一种单向通信模型，发布方发布数据，订阅方订阅数据，数据流单向的由发布方传输到订阅方。

话题通信的发布方和订阅方是一种多对多的关系。

ros2 pkg create cpp01_topic --build-type ament_cmake --dependencies rclcpp std_msgs base_interfaces --node_name demo01_talker_str_cpp

ros2 pkg create py01_topic --build-type ament_python --dependencies rclpy std_msgs base_interfaces --node_name demo01_talker_str_py

验证发布方是否将话题数据发送出去：ros2 topic echo /topic，其中topic为话题名称。


#### 服务通信

一种基于请求响应的通信模型，在通信双方中，客户端发送请求数据到服务端，服务端响应结果给客户端。

#### 动作通信

一种带有连续反馈的通信模型，在通信双方中，客户端发送请求数据到服务端，服务端响应结果给客户端，但是在服务端接收到请求到产生最终响应的过程中，会发送连续的反馈信息到客户端。

#### 参数通信（基于服务通信）

一种基于数共享的通信模型，在通信双方中，服务端可以设置数据，而客户端可以连接服务端并操作服务端数据。

### 接口

msg文件是用于定义话题通信中数据载体的接口文件。

srv文件是用于定义服务通信中数据载体的接口文件。

action文件是用于定义动作通信中数据载体的接口文件。