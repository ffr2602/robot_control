import socket

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


IPAddr = socket.gethostbyname(socket.gethostname())


def generate_launch_description():

    port = LaunchConfiguration('port')
    address = LaunchConfiguration('address')
    ssl = LaunchConfiguration('ssl')
    certfile = LaunchConfiguration('certfile')
    keyfile = LaunchConfiguration('keyfile')

    retry_startup_delay = LaunchConfiguration('retry_startup_delay')

    fragment_timeout = LaunchConfiguration('fragment_timeout')
    delay_between_messages = LaunchConfiguration('delay_between_messages')
    max_message_size = LaunchConfiguration('max_message_size')
    unregister_timeout = LaunchConfiguration('unregister_timeout')

    use_compression = LaunchConfiguration('use_compression')
    call_services_in_new_thread = LaunchConfiguration('call_services_in_new_thread')

    topics_glob = LaunchConfiguration('topics_glob')
    services_glob = LaunchConfiguration('services_glob')
    params_glob = LaunchConfiguration('params_glob')
    bson_only_mode = LaunchConfiguration('bson_only_mode')

    use_ssl = Node(
        condition=IfCondition(ssl),
        name='rosbridge_websocket',
        package='rosbridge_server',
        executable='rosbridge_websocket',
        output='screen',
        parameters=[{
            'certfile': certfile,
            'keyfile': keyfile,
            'port': port,
            'address': address,
            'retry_startup_delay': retry_startup_delay,
            'fragment_timeout': fragment_timeout,
            'delay_between_messages': delay_between_messages,
            'max_message_size': max_message_size,
            'unregister_timeout': unregister_timeout,
            'use_compression': use_compression,
            'call_services_in_new_thread': call_services_in_new_thread,

            'topics_glob': topics_glob,
            'services_glob': services_glob,
            'params_glob': params_glob
        }]
    )

    not_use_ssl = Node(
        condition=UnlessCondition(ssl),
        name='rosbridge_websocket',
        package='rosbridge_server',
        executable='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': port,
            'address': address,
            'retry_startup_delay': retry_startup_delay,
            'fragment_timeout': fragment_timeout,
            'delay_between_messages': delay_between_messages,
            'max_message_size': max_message_size,
            'unregister_timeout': unregister_timeout,
            'use_compression': use_compression,
            'call_services_in_new_thread': call_services_in_new_thread,

            'topics_glob': topics_glob,
            'services_glob': services_glob,
            'params_glob': params_glob,

            'bson_only_mode': bson_only_mode
        }]
    )

    rospi_node = Node(
        name='rosapi',
        package='rosapi',
        executable='rosapi_node',
        parameters=[{
            'topics_glob': topics_glob,
            'services_glob': services_glob,
            'params_glob': params_glob,
        }]
    )

    return LaunchDescription([

        DeclareLaunchArgument('port', default_value='9090'),
        DeclareLaunchArgument('address', default_value=IPAddr),
        DeclareLaunchArgument('ssl', default_value='false'),
        DeclareLaunchArgument('certfile', default_value=''),
        DeclareLaunchArgument('keyfile', default_value=''),

        DeclareLaunchArgument('retry_startup_delay', default_value='5.0'),

        DeclareLaunchArgument('fragment_timeout', default_value='600'),
        DeclareLaunchArgument('delay_between_messages', default_value='0'),
        DeclareLaunchArgument('max_message_size', default_value='10000000'),
        DeclareLaunchArgument('unregister_timeout', default_value='10.0'),

        DeclareLaunchArgument('use_compression', default_value='false'),
        DeclareLaunchArgument('call_services_in_new_thread', default_value='false'),

        DeclareLaunchArgument('topics_glob', default_value=''),
        DeclareLaunchArgument('services_glob', default_value=''),
        DeclareLaunchArgument('params_glob', default_value=''),
        DeclareLaunchArgument('bson_only_mode', default_value='false'),


        use_ssl,
        not_use_ssl,
        rospi_node
    ])
