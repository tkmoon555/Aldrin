import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    package_dir = get_package_share_directory("aldrin_sim")
    xacro_path = os.path.join(package_dir,"simplebot.xacro")
    urdf_path = os.path.join(package_dir,'simplebot.urdf')
    config_file_name = "simplebot.rviz"


    # xacroをロード
    doc = xacro.process_file(xacro_path)
# xacroを展開してURDFを生成
    robot_desc = doc.toprettyxml(indent='  ')
    # urdf_pathに対してurdfを書き出し
    f = open(urdf_path, 'w')
    f.write(robot_desc)
    f.close()

    rviz_config = os.path.join(
        package_dir,
        config_file_name)

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

       Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf_path]),
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'),
        
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]
            ),

    ])