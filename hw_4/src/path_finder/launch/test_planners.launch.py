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
    pkg_grid_path_searcher = get_package_share_directory('path_finder')
    rviz2_config = os.path.join(pkg_grid_path_searcher, "launch", "traj.rviz")

    # arguments
    test_case = LaunchConfiguration('test_case')
    map_size_x = LaunchConfiguration('map_size_x')
    map_size_y = LaunchConfiguration('map_size_y')
    map_size_z = LaunchConfiguration('map_size_z')
    origin_x = LaunchConfiguration('origin_x')
    origin_y = LaunchConfiguration('origin_y')
    origin_z = LaunchConfiguration('origin_z')
    # declare the launch arguments
    declare_test_case = DeclareLaunchArgument(
        'test_case',
        default_value= 'rrt_star',
        description=('test_case'))
    declare_map_size_x = DeclareLaunchArgument(
        'map_size_x',
        default_value= '50.0',
        description=('map_size_x'))
    declare_map_size_y = DeclareLaunchArgument(
        'map_size_y',
        default_value= '50.0',
        description=('map_size_y'))
    declare_map_size_z = DeclareLaunchArgument(
        'map_size_z',
        default_value= '8.0',
        description=('map_size_z'))
    declare_origin_x = DeclareLaunchArgument(
        'origin_x',
        default_value= '-25.0',
        description='origin_x')
    declare_origin_y = DeclareLaunchArgument(
        'origin_y',
        default_value= '-25.0',
        description='origin_y')
    declare_origin_z = DeclareLaunchArgument(
        'origin_z',
        default_value= '-1.0',
        description='origin_z')
    # nodes
    random_forest = Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        parameters=[
            {'init_state_x': 0.0,
             'init_state_y': 0.0,
             'map.x_size': map_size_x,
             'map.y_size': map_size_y,
             'map.z_size': map_size_z,
             'map.circle_num': 100,
             'map.obs_num': 120,
             'map.resolution': 0.1,
             'ObstacleShape.lower_rad': 0.4,
             'ObstacleShape.upper_rad': 2.5,
             'ObstacleShape.lower_hei': 0.5,
             'ObstacleShape.upper_hei': 7.5,
             'CircleShape.lower_circle_rad': 0.9,
             'CircleShape.upper_circle_rad': 3.2,
             'sensing.rate': 1.0}],
        output='screen'
    )
    path_finder_node = Node(
        package='path_finder',
        executable='path_finder',
        name='path_finder_node',
        parameters=[
            {'test_case': test_case,
             'occ_map.resolution': 0.5,
             'occ_map.map_size_x': map_size_x,
             'occ_map.map_size_y': map_size_y,
             'occ_map.map_size_z': map_size_z,
             'occ_map.origin_x': -25.0,
             'occ_map.origin_y': -25.0,
             'occ_map.origin_z': -1.0,
             'RRT_Star.steer_length': 2.0,
             'RRT_Star.search_radius': 6.0,
             'RRT_Star.search_time': 4.0,
             'RRT_Star.max_tree_node_nums': 5000}],
        remappings=[
                ('global_cloud', 'global_map')],
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
    ld.add_action(declare_test_case)
    ld.add_action(declare_map_size_x)
    ld.add_action(declare_map_size_y)
    ld.add_action(declare_map_size_z)
    ld.add_action(declare_origin_x)
    ld.add_action(declare_origin_y)
    ld.add_action(declare_origin_z)
    ld.add_action(random_forest)
    ld.add_action(path_finder_node)
    ld.add_action(rviz2)
    return ld