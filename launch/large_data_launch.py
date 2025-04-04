from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Talkerノードの設定
    talker_node = Node(
        package='your_package_name',  # パッケージ名を実際のものに変更してください
        executable='large_talker_node',
        name='talker_node',
        output='screen'
    )

    # Listenerノードの設定
    listener_node = Node(
        package='your_package_name',  # パッケージ名を実際のものに変更してください
        executable='large_listener_node',
        name='listener_node',
        output='screen'
    )

    # 両方のノードを含むLaunchDescription
    return LaunchDescription([
        talker_node,
        listener_node
    ])
