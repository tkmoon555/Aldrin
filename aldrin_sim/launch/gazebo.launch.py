# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Constants for paths to different files and folders
    gazebo_pkg_name = 'aldrin_sim'
    world_file_path = 'worlds/basic.world'
    # Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '1.0'
    spawn_yaw_val = '0.0'

    gazebo_pkg_share = FindPackageShare(package=gazebo_pkg_name).find(gazebo_pkg_name)
    world_path = os.path.join(gazebo_pkg_share, world_file_path)

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                    launch_arguments={'world': world_path}.items(),
             )

    aldrin_sim_path = os.path.join(
        get_package_share_directory('aldrin_sim'))

    xacro_file = os.path.join(aldrin_sim_path,
                              'simplebot.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'use_sim_time': use_sim_time,'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'aldrin',
                                   '-x', spawn_x_val,
                                   '-y', spawn_y_val,
                                   '-z', spawn_z_val,
                                   '-Y', spawn_yaw_val
                                   ],
                        output='screen')

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

#    load_joint_trajectory_controller = ExecuteProcess(
#        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'effort_controllers'],
#        output='screen'
#    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
#        RegisterEventHandler(
#            event_handler=OnProcessExit(
#                target_action=load_joint_state_controller,
#                on_exit=[load_joint_trajectory_controller],
#            )
#        ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])