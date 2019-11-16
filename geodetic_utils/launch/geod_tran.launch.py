import launch
import launch_ros
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    # use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default=False)

    return launch.LaunchDescription(
        [
            # launch_ros.actions.Node(package='geodetic_utils', node_executable='talker'),
            launch_ros.actions.Node(package='geodetic_utils', node_executable='set_gps_reference_node', output='screen'),
            launch_ros.actions.Node(package='geodetic_utils', node_executable='gps_to_pose_conversion_node'),
            
        ])