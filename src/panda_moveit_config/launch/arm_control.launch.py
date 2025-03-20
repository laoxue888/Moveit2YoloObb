#!/usr/bin/env python3

from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import os
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess


def generate_launch_description():
    """
    说明：由于gazebo无法启动调试，因此将部分节点放到这里来启动，以实现arm_control_from_UI.py文件的调试
    """
    arm_robot_sim_path = os.path.join(get_package_share_directory('panda_moveit_config'))
    
    moveit_config = (
        MoveItConfigsBuilder(robot_name="panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .moveit_cpp(arm_robot_sim_path + "/config/controller_setting.yaml")
        .to_moveit_configs()
    )
    #.joint_limits(file_path="config/joint_limits.yaml")
    #.robot_description_kinematics(file_path="config/kinematics.yaml")

    moveit_py_node = Node(
        name="moveit_py",
        package="panda_moveit_config",
        executable="arm_control_from_UI.py",
        output="both",
        parameters=[moveit_config.to_dict(),
                    {"use_sim_time": True},
                    ],
    )

    # rviz_config_file = os.path.join(
    #     get_package_share_directory("panda_moveit_config"),
    #     "config",
    #     "motion_planning.rviz",
    # )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    #     parameters=[
    #         moveit_config.robot_description,
    #         moveit_config.robot_description_semantic,
    #         {"use_sim_time": True},
    #     ],
    # )

    # static_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=["--frame-id", "world", "--child-frame-id", "panda_link0"],
    #     parameters=[{"use_sim_time": True},],
    # )

    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     output="log",
    #     parameters=[moveit_config.robot_description,
    #                 {"use_sim_time": True},
    #                 ],
    # )

    # # Bridge
    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=['/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'], 
    #     output='screen'
    # )

    # load_controllers = []
    # for controller in [
    #     "panda_arm_controller",
    #     "panda_hand_controller",
    #     "joint_state_broadcaster",
    # ]:
    #     load_controllers += [
    #         ExecuteProcess(
    #             cmd=["ros2 run controller_manager spawner {}".format(controller)],
    #             shell=True,
    #             output="log",
    #         )
    #     ]

    return LaunchDescription(
        [
            moveit_py_node,
            # robot_state_publisher,
            # bridge,
            # rviz_node,
            # static_tf,
        ]
        # + load_controllers
    )