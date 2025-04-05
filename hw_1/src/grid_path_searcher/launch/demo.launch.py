# Copyright gezp
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # config
    pkg_grid_path_searcher = get_package_share_directory('grid_path_searcher')
    rviz2_config = os.path.join(pkg_grid_path_searcher, "launch", "demo.rviz")
    # arguments
    map_size_x = LaunchConfiguration('map_size_x')
    map_size_y = LaunchConfiguration('map_size_y')
    map_size_z = LaunchConfiguration('map_size_z')
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    start_z = LaunchConfiguration('start_z')
    # declare the launch arguments
    declare_map_size_x = DeclareLaunchArgument(
        'map_size_x',
        default_value= '20.0',
        description=('map_size_x'))
    declare_map_size_y = DeclareLaunchArgument(
        'map_size_y',
        default_value= '20.0',
        description=('map_size_y'))
    declare_map_size_z = DeclareLaunchArgument(
        'map_size_z',
        default_value= '10.0',
        description=('map_size_z'))
    declare_start_x = DeclareLaunchArgument(
        'start_x',
        default_value= '0.0',
        description='start_x')
    declare_start_y = DeclareLaunchArgument(
        'start_y',
        default_value= '0.0',
        description='start_y')
    declare_start_z = DeclareLaunchArgument(
        'start_z',
        default_value= '1.0',
        description='start_z')
    # map_size_x=20.0
    # map_size_y=20.0
    # map_size_z= 10.0
    # start_x= 0.0
    # start_y= 0.0
    # start_z= 1.
    # nodes
    demo_node = Node(
        package='grid_path_searcher',
        executable='demo_node',
        name='demo_node',
        parameters=[
            {'map.margin': 0.,
             'map.resolution': 0.2,
             'map.x_size': map_size_x,
             'map.y_size': map_size_y,
             'map.z_size': map_size_z,
             'planning.start_x': start_x,
             'planning.start_y': start_y,
             'planning.start_z': start_z}],
        remappings=[
                ('map', 'global_map')],
        output='screen'
    )
    random_complex = Node(
        package='grid_path_searcher',
        executable='random_complex',
        name='random_complex',
        parameters=[
            {'init_state_x': start_x,
             'init_state_y': start_y,
             'map.x_size': map_size_x,
             'map.y_size': map_size_y,
             'map.z_size': map_size_z,
             'map.circle_num': 40,
             'map.obs_num': 300,
             'map.resolution': 0.1,
             'ObstacleShape.lower_rad': 0.1,
             'ObstacleShape.upper_rad': 0.7,
             'ObstacleShape.lower_hei': 1.0,
             'ObstacleShape.upper_hei': 3.0,
             'CircleShape.lower_circle_rad': 0.6,
             'CircleShape.upper_circle_rad': 2.0,
             'sensing.rate': 0.5}],
        output='screen'
    )
    waypoint_generator = Node(
        package='waypoint_generator',
        executable='waypoint_generator',
        name='waypoint_generator',
        parameters=[
            {'waypoint_type': 'manual-lonely-waypoint'}],
        output='screen'
    )
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz2_config]
    )
    ld = LaunchDescription()
    ld.add_action(declare_map_size_x)
    ld.add_action(declare_map_size_y)
    ld.add_action(declare_map_size_z)
    ld.add_action(declare_start_x)
    ld.add_action(declare_start_y)
    ld.add_action(declare_start_z)
    ld.add_action(demo_node)
    ld.add_action(random_complex)
    ld.add_action(waypoint_generator)
    ld.add_action(rviz2)
    return ld