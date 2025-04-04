from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='large_data_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='pubsub_test',
                plugin='large_data_pub::TalkerComponent',
                name='talker_node'),
            ComposableNode(
                package='pubsub_test',
                plugin='large_data_sub::ListenerComponent',
                name='listener_node'),
        ],
        output='screen',
    )

    return LaunchDescription([container])
