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
    pkg_trajectory_optimization = get_package_share_directory('trajectory_optimization')
    click_gen_params = os.path.join(pkg_trajectory_optimization, "config", "click_gen.yaml")
    rviz2_config = os.path.join(pkg_trajectory_optimization, "launch", "click_gen.rviz")
    # nodes
    click_gen_node = Node(
        package='trajectory_optimization',
        executable='click_gen',
        name='click_gen_node',
        parameters=[click_gen_params],
        output='screen'
    )
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz2_config],
        output="screen"
    )
    ld = LaunchDescription()
    ld.add_action(click_gen_node)
    ld.add_action(rviz2)
    return ld