import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_car_simulator = get_package_share_directory('car_simulator')
    pkg_mpc_car = get_package_share_directory('mpc_car')
    car_simulator_config = os.path.join(pkg_car_simulator, 'config', 'car_simulator.yaml')
    mpc_car_config = os.path.join(pkg_mpc_car, 'config', 'mpc_car.yaml')
    rviz2_config = os.path.join(pkg_car_simulator, "launch", "rviz_sim.rviz")
    container = Node(
        name='container',
        package='rclcpp_components',
        executable='component_container_isolated',
        output='screen'
    )
    load_nodes = LoadComposableNodes(
        target_container='container',
        composable_node_descriptions=[
            ComposableNode(
                package='car_simulator',
                plugin='car_simulator::CarSimulator',
                name='car_simulator',
                parameters=[car_simulator_config]),
            ComposableNode(
                package='mpc_car',
                plugin='mpc_car::MpcCarNode',
                name='mpc_car',
                parameters=[mpc_car_config])       
        ]
    )
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz2_config],
        output="screen"
    )
    ld = LaunchDescription()
    ld.add_action(container)
    ld.add_action(load_nodes)
    ld.add_action(rviz2)
    return ld
