from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # コンテナ1（Talker用）
    talker_container = ComposableNodeContainer(
        name='talker_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='pubsub_test',
                plugin='large_data_pub::TalkerComponent',
                name='talker_node'),
        ],
        output='screen',
    )

    # コンテナ2（Listener用）
    listener_container = ComposableNodeContainer(
        name='listener_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='pubsub_test',
                plugin='large_data_sub::ListenerComponent',
                name='listener_node'),
        ],
        output='screen',
    )

    return LaunchDescription([
        talker_container,
        listener_container
    ])
