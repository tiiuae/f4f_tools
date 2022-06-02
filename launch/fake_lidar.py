import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "f4f_tools"
    pkg_share_path = get_package_share_directory(pkg_name)
 
    ld.add_action(launch.actions.DeclareLaunchArgument("use_sim_time", default_value="false"))

    DRONE_DEVICE_ID=os.getenv('DRONE_DEVICE_ID')

    namespace=DRONE_DEVICE_ID
    ld.add_action(
        Node(
            package=pkg_name,
            executable='talker',
            name='fake_lidar_publisher',
            remappings=[
                ('topic', DRONE_DEVICE_ID + '/rplidar/scan'),
            ],
            output='screen',
            parameters=[
                {"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},
                {"frame_id": DRONE_DEVICE_ID + '/rplidar'},
                {"rate": 100.0},
                {"blocking_distance": 3.0},
            ],
        ),
    )

    return ld
