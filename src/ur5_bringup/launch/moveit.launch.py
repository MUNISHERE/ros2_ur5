#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    is_sim = LaunchConfiguration('is_sim')
    
    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        default_value='True'
    )
    initial_positions_file_path = os.path.join(
        get_package_share_directory('ur5_bringup'),
        'config',
        'ur5',
        'initial_positions.yaml'
    )
    moveit_config = (
        MoveItConfigsBuilder("ur5", package_name="ur5_bringup")
        .trajectory_execution(file_path="config/moveit_controller.yaml")
        .robot_description(file_path=os.path.join(
            get_package_share_directory("ur5_description"),
            "urdf",
            "ur5.urdf.xacro"
            )
        )
        .robot_description_semantic(file_path="config/ur5.srdf")
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "chomp"],
            default_planning_pipeline="ompl"
        )
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .to_moveit_configs()
    )

        # Create move_group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': is_sim},
            {'start_state': {'content': initial_positions_file_path}},
            {'publish_robot_description_semantic': True},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

        # RViz
    #rviz_config = os.path.join(
    #    get_package_share_directory("ur5_bringup"),
    #        "config",
    #        "moveit.rviz",
    #)
    #rviz_node = Node(
        #package="rviz2",
        #executable="rviz2",
        #name="rviz2",
        #output="log",
        #arguments=["-d", rviz_config],
        #parameters=[
            #moveit_config.robot_description,
            #moveit_config.robot_description_semantic,
            #moveit_config.robot_description_kinematics,
            #moveit_config.joint_limits,
        #],
    #)

    return LaunchDescription(
        [
            is_sim_arg,
            move_group_node, 
            #rviz_node
        ]
    )