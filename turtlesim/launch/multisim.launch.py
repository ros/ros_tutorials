from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            node_namespace= "turtlesim1", package='turtlesim', node_executable='turtlesim_node', output='screen'),
        launch_ros.actions.Node(
            node_namespace= "turtlesim2", package='turtlesim', node_executable='turtlesim_node', output='screen'),
    ])
