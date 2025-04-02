#!/usr/bin/env python3
#
# Copyright (c) 2024 SICK AG, Waldkirch
# SPDX-License-Identifier: Unlicense

"""
This script is used to launch ROS2 nodes for the Visionary cameras.
The script defines a launch description for a ComposableNodeContainer that runs
the VisionaryPublisher plugin. The publishers are configured with the serial
number of the cameras. The node is executed in an isolated component container
with a multi-threaded executor.

The node publishes data from the Visionary camera to the ROS2 ecosystem, 
allowing other nodes to consume and process the camera data.
"""
import os
from typing import Dict, List

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def read_serial_numbers(params_yaml_fp: str) -> Dict[str, List[str]]:
    """
    Reads serial numbers from a YAML file and returns them as a dictionary
    with the model name as the key and the list of serial numbers as the value.

    Args:
        params_yaml_fp (str): The path to the YAML file containing serial numbers.

    Returns:
        Dict[str, List[str]]: A dictionary with model names as keys and lists of serial numbers as values.
    """
    with open(params_yaml_fp, 'r') as yaml_file:
        params = yaml.load(yaml_file, Loader=yaml.FullLoader)
        serials_dict = params["serial_numbers"]
        model_serials = {}
        for camera_model, serial_list in serials_dict.items():
            model_serials[camera_model] = [
                serial for serial in serial_list if serial]
    return model_serials

def add_visionary_pub_to_launch_description(launch_description: LaunchDescription, model: str, serial: str):
    """
    Adds the visionary publisher launch description to the given launch description.

    Args:
        launch_description (LaunchDescription): The launch description to which the entity will be added.
        model (str): The model name to be used in the namespace.
        serial (str): The serial number of the device.

    Returns:
        None
    """
    launch_description.add_entity(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sick_visionary_gev_ros2'),
                'launch',
                'visionary_publisher_launch.py'
            ])
        ]),
        launch_arguments={
            'node_name': f'cam_{serial}',
            'name_space': '/' + model,
            'serial_number': TextSubstitution(text=str(serial)),
        }.items()
    ))

def add_pointcloud_pub_to_launch_description(launch_description: LaunchDescription, publish_pointcloud: LaunchConfiguration, model: str, serial: str):
    """
    Adds the pointcloud publisher launch description to the given launch description.

    Args:
        launch_description (LaunchDescription): The launch description to which the entity will be added.
        publish_pointcloud (LaunchConfiguration): The condition to check if the pointcloud publisher should be included.
        model (str): The model name to be used in the namespace.
        serial (str): The serial number of the device.

    Returns:
        None
    """
    launch_description.add_entity(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sick_visionary_gev_ros2'),
                'launch',
                'pointcloud_publisher_launch.py'
            ])
        ]),
        launch_arguments={
            'node_name': f'pointcloud_publisher_{serial}',
            'name_space': '/' + model,
            'serial_number': TextSubstitution(text=str(serial)),
        }.items(),
        condition=IfCondition(publish_pointcloud)
    ))

def create_robot_state_pub_description(model: str, serial: str) -> ComposableNode:
    """
    Creates a robot state publisher description for a given model and serial number.

    Args:
        model (str): The model name of the robot.
        serial (str): The serial number of the device.

    Returns:
        ComposableNode: A ComposableNode object configured for the robot state publisher.
    """
    xacro_file_name = model + ".xacro"
    xacro_path = os.path.join(get_package_share_directory(
        'sick_visionary_gev_ros2'), 'meshes', xacro_file_name)
    robot_description_config = xacro.process_file(
        xacro_path, mappings={"link_name": "camera_frame_" + serial}).toxml()
    robot_description = {'robot_description': robot_description_config}
    return ComposableNode(
        package='robot_state_publisher',
        plugin='robot_state_publisher::RobotStatePublisher',
        namespace='/' + model + '/' + f'cam_{serial}',
        name=f'robot_state_publisher',
        parameters=[robot_description]
    )

def generate_launch_description():
    """
    This function generates the launch description 
    for the Visionary camera node.

    Returns:
        LaunchDescription: A ROS2 LaunchDescription object that 
        describes the node to be launched.
    """

    # Launch arguments
    publish_pointcloud_arg = DeclareLaunchArgument(
        'publish_pointcloud',
        default_value='true',
        description='Whether to publish point cloud data'
    )
    publish_pointcloud = LaunchConfiguration('publish_pointcloud')
   
    # Load param.yaml file
    config = os.path.join(
        get_package_share_directory('sick_visionary_gev_ros2'),
        'config',
        'params.yaml'
    )    
    models_serials_dict = read_serial_numbers(config)
    
    # Launch description
    launch_description = LaunchDescription()    
    launch_description.add_entity(publish_pointcloud_arg)
    robot_state_publisher_nodes = []
    for model, serials in models_serials_dict.items():
        for serial in serials:
            add_visionary_pub_to_launch_description(launch_description, model, serial)
            add_pointcloud_pub_to_launch_description(launch_description, publish_pointcloud, model, serial)
            robot_state_publisher_nodes.append(create_robot_state_pub_description(model, serial))
    
    # Add entities to launch description
    container = ComposableNodeContainer(
            name='visionary_cameras',
            namespace='sick',
            package='rclcpp_components',
            executable='component_container_isolated',
            arguments=["--use_multi_threaded_executor"],
            output='screen',
            composable_node_descriptions=robot_state_publisher_nodes,
        )
    launch_description.add_entity(container)

    return launch_description
