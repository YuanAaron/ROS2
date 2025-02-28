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

