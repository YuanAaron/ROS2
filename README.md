## 项目创建、编译、运行

### C++版
1. 进入工作空间的src目录创建功能包：ros2 pkg create 01pkg_helloworld_cpp --build-type ament_cmake --dependencies rclcpp --node-name helloworld
2. 在工作空间目录：colcon build
3. 在工作空间目录：
	+ . install/setup.bash 或 source install/setup.bash（否则在其他终端下无法运行）
	+ ros2 run 01pkg_helloworld_cpp helloworld

### Python版
1. 进入工作空间的src目录创建功能包：ros2 pkg create 02pkg_helloworld_py --build-type ament_python --dependencies rclpy --node-name helloworld
2. 在工作空间目录：colcon build
3. 在工作空间目录：
	+ . install/setup.bash 或 source install/setup.bash(同理）
	+ ros2 run 02pkg_helloworld_py helloworld
