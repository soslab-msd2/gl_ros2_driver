import os
import launch

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    rviz_config_file = os.path.join(get_package_share_directory('gl_ros2_driver'), 'rviz', 'config.rviz')
    
    ld = launch.LaunchDescription()

    gl_ros2_driver_node = Node(
        node_name = 'gl_ros2_driver_node',
        package = 'gl_ros2_driver',
        node_executable = 'gl_ros2_driver_node',
        output = 'screen',
        parameters = [
            {'serial_port_name': '/dev/ttyUSB0'},
            {'serial_baudrate': 921600},
            {'frame_id': 'laser'},
            {'parent_frame_id': 'odom'},
            {'pub_topicname_lidar': 'scan'},
        ],
    )

    rviz_node = Node(
        node_name = 'rviz2',
        package = 'rviz2',
        node_executable = 'rviz2',
        output = 'screen',
        arguments = ['-d', rviz_config_file],
    )

    ld.add_action( gl_ros2_driver_node )
    ld.add_action( rviz_node )
    
    return ld
