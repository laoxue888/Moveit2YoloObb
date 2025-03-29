
---

# 前言(Introduction)

YOLO机械臂仿真是一种结合了YOLO（You Only Look Once）目标检测算法与机械臂控制的智能化仿真系统。该系统通过YOLO实时检测环境中的目标物体，并结合机械臂的运动规划与仿真，实现自动化的抓取、分拣、装配等任务。该应用广泛应用于工业自动化、智能仓储、机器人教育等领域，为机械臂的智能化操作提供了高效、精准的解决方案。

在仿真环境中，YOLO算法能够快速识别目标物体的位置和类别，并将这些信息传递给机械臂控制系统。机械臂根据目标的位置和姿态，自动规划运动路径，完成抓取或操作任务。通过仿真平台（如Gazebo、CoppeliaSim或PyBullet），用户可以在虚拟环境中测试和优化机械臂的控制算法，降低实际部署中的风险和成本。

YOLO机械臂仿真的优势在于其高效的目标检测能力和实时性，能够适应动态环境中的多目标场景。同时，仿真平台提供了高度可控的实验环境，便于开发者调试算法、优化性能，并快速验证机械臂的工作效果。

![](https://github.com/laoxue888/moveit2_yolobb_ws/blob/main/src/_docs/images/d7802cf1-4a28-40c1-a918-37d22d51f752.gif)

> 参考：
> - [Pick and Place Simulation Using MoveIt and Yolov8 OBB](https://www.youtube.com/watch?v=ypr3RtJzgKI)
> - [https://drive.google.com/drive/folders/1eDVATIX1mHBtSkI7ueQE2568mkxgno0f](https://drive.google.com/drive/folders/1eDVATIX1mHBtSkI7ueQE2568mkxgno0f)

[【代码】基于ros2与moveit2开发的yolo识别抓取虚拟机械臂](https://www.bilibili.com/video/BV1KqXWYHE6k/?vd_source=3bf4271e80f39cfee030114782480463)

# 环境配置(Environment configuration)

> - Ubuntu:24.04
> - ros2:jazzy

❇️创建Docker容器(Create a docker container)
```shell
docker run -it -p 6796:22 -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=host.docker.internal:0.0 --gpus=all --name=ros2_learn6 docker.1ms.run/ubuntu:24.04  /bin/bash
```

❇️进入Docker容器，配置开发环境(Enter the Docker container and configure the development environment)
```shell
# 按照鱼香ros一键安装ros2
apt-get update
apt install wget -y
wget http://fishros.com/install -O fishros && bash fishros

# 打开新的终端，安装gz
sudo apt-get update
sudo apt-get install curl lsb-release gnupg -y
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update -y
sudo apt-get install gz-harmonic -y

# 安装远程显示服务程序
apt-get install x11-xserver-utils
apt install libxcb* -y

# 安装moveit
apt install ros-${ROS_DISTRO}-moveit* -y

# sudo apt-get update 
# sudo apt-get install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp 
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export LIBGL_ALWAYS_SOFTWARE=1
# export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/${ROS_DISTRO}/lib/
# # sudo apt install ros-${ROS_DISTRO}-gazebo-ros-pkgs ros-${ROS_DISTRO}-gazebo-ros2-control -y

# 安装ros2的控制功能包
sudo apt install ros-${ROS_DISTRO}-controller-manager -y
sudo apt install ros-${ROS_DISTRO}-joint-trajectory-controller -y
sudo apt install ros-${ROS_DISTRO}-joint-state-broadcaster -y
sudo apt install ros-${ROS_DISTRO}-diff-drive-controller -y

# 安装其他功能包
apt install ros-${ROS_DISTRO}-ros-gz -y
apt-get install ros-${ROS_DISTRO}-joint-state-publisher-gui -y
apt install ros-${ROS_DISTRO}-moveit-ros-planning-interface -y
# apt install ros-jazzy-gz-ros2-control 这个很重要 https://github.com/ros-controls/gz_ros2_control
apt install ros-${ROS_DISTRO}-gz-ros2-control -y

# 用于调试，可不安装
apt-get install gdb -y

# 安装python第三方库
apt install python3-pip -y
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
# pip install pyside6 xacro ultralytics --break-system-packages
pip install pyside6 xacro ultralytics NodeGraphQt --break-system-packages
pip install -U colcon-common-extensions vcstool --break-system-packages
```

# 运行测试(Run test)

```shell
# Shell A
source install/setup.bash
ros2 launch panda_moveit_config gazebo_obb.launch.py

# Shell B 调试用，在vscode中要安装ROS（Microsoft）、Python等模块，还要在Python文件中选择Python编译器
# source install/setup.bash
# ros2 launch panda_moveit_config arm_control.launch.py

# Shell C
source install/setup.bash
ros2 launch yolov8_obb yolov8_obb.launch.py

# Shell D
source install/setup.bash
cd src/ui_controller/
python3 main.py
```

# ros2基本操作

```shell
# 创建文件夹
ros2 pkg create pkg_demo --node-name helloworld_node --build-type ament_python --dependencies rclpy std_msgs
```

# moveit测试

![alt text](src/_docs/images/image-2.png)


# 编译调试

```shell
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

# 报错

❌

```shell
[move_group-3] [ERROR] [1742286584.537923487] [move_group.moveit.moveit.core.time_optimal_trajectory_generation]: No acceleration limit was defined for joint panda_joint1! You have to define acceleration limits in the URDF or joint_limits.yaml
```

✔️

设置好加速度

![alt text](src/_docs/images/image-1.png)

❌

```shell
[move_group-3] [ERROR] [1742286725.635634628] [move_group.moveit.moveit.ros.trajectory_execution_manager]: Unable to identify any set of controllers that can actuate the specified joints: [ panda_joint1 panda_joint2 panda_joint3 panda_joint4 panda_joint5 panda_joint6 panda_joint7 ]
[move_group-3] [ERROR] [1742286725.635688171] [move_group.moveit.moveit.ros.trajectory_execution_manager]: Known controllers and their joints:
[move_group-3] 
[move_group-3] [ERROR] [1742286725.635703240] [move_group.moveit.moveit.ros.plan_execution]: Apparently trajectory initialization failed
[move_group-3] [INFO] [1742286725.635767383] [move_group.moveit.moveit.ros.move_group.move_action]: CONTROL_FAILED
[rviz2-4] [INFO] [1742286725.636182552] [moveit_2565687607.moveit.ros.move_group_interface]: Plan and Execute request aborted
[rviz2-4] [ERROR] [1742286725.637270976] [moveit_2565687607.moveit.ros.move_group_interface]: MoveGroupInterface::move() failed or timeout reached
```

✔️

![alt text](src/_docs/images/image.png)

```shell
action_ns: follow_joint_trajectory
default: true
```

❌：执行夹取的过程中，夹爪打开和关闭异常

```shell
[ERROR] [1742603349.012384448] [moveit_3836862178.moveit.ros.check_start_state_bounds]: Joint 'panda_finger_joint1' from the starting state is outside bounds by: [-6.61565e-14 ] should be in the range [0 ], [0.04 ].
[ERROR] [1742603349.012503158] [moveit_py]: PlanningRequestAdapter 'CheckStartStateBounds' failed, because 'Start state out of bounds.'. Aborting planning pipeline.
[ERROR] [1742603349.013678982] [moveit_py.pose_goal]: Planning failed
```

🤔：范围超了，导致`CheckStartStateBounds`不通过

✔️：调试一下初始位置
