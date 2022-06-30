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
            executable='gps_republisher',
            name='gps_republisher',
            remappings=[
                ('gps_topic_in', '/' + DRONE_DEVICE_ID + '/fmu/vehicle_gps_position/out'),
                ('gps_topic_out', '/' + DRONE_DEVICE_ID + '/mesh/vehicle_gps_position/out'),
                ('local_topic_in', '/' + DRONE_DEVICE_ID + '/fmu/vehicle_local_position/out'),
                ('local_topic_out', '/' + DRONE_DEVICE_ID + '/mesh/vehicle_local_position/out'),
            ],
            output='screen',
            parameters=[
                {"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},
            ],
        ),
    )

    return ld
