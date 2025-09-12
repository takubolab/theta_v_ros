from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch 引数
    image_raw_on_arg = DeclareLaunchArgument(
        'image_raw_on',  # ここはスラッシュなし
        default_value='true'
    )
    image_compress_on_arg = DeclareLaunchArgument(
        'image_compress_on',
        default_value='true'
    )
    use4k_arg = DeclareLaunchArgument(
        'use4k',
        default_value='false'
    )

    # Node
    theta_node = Node(
        package='theta_v_ros',
        executable='driver',
        name='thetaV',
        output='screen',
        respawn=True,
        parameters=[{
            'nvdec': False,
            'use4k': LaunchConfiguration('use4k'),
            'image/raw/on': LaunchConfiguration('image_raw_on'),
            'image/compress/on': LaunchConfiguration('image_compress_on'),
            'image/compress/level': 2
        }]
    )
    
    return LaunchDescription([
        image_raw_on_arg,
        image_compress_on_arg,
        use4k_arg,
        theta_node,
    ])
