import os
import launch

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    gl_ros2_driver = Node(
        node_name = 'gl_ros2_driver',
        package = 'gl_ros2_driver',
        node_executable = 'gl_ros2_driver_node',
        output = 'screen',
        parameters = [
            {'serial_port_name': '/dev/ttyUSB0'},
            {'serial_baudrate': 921600},
            {'frame_id': 'laser'},
            {'pub_topicname_lidar': 'scan'},
            {'angle_offset': 0.0},
        ],
    )

    ld = launch.LaunchDescription()
    ld.add_action( gl_ros2_driver )
    
    return ld
