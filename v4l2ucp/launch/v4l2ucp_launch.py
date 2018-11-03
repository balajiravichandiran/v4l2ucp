# Copyright 2018 Lucas Walter

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    usb_cam_dir = get_package_share_directory('usb_cam')
    v4l2ucp_dir = get_package_share_directory('v4l2ucp')
    print('usb_cam dir ' + usb_cam_dir)
    # TODO(lucasw) use the usb_cam launch file instead of duplicating
    launches = []
    launches.append(launch_ros.actions.Node(
            package='usb_cam', node_executable='usb_cam_node', output='screen',
            arguments=["__params:=" + usb_cam_dir + "/config/params.yaml"]
            ))
    launches.append(launch_ros.actions.Node(
            package='v4l2ucp', node_executable='v4l2ucp_node', output='screen',
            arguments=["__params:=" + v4l2ucp_dir + "/config/params.yaml"]
            ))

    launches.append(launch_ros.actions.Node(
            package='usb_cam', node_executable='show_image.py', output='screen',
            # arguments=[image_manip_dir + "/data/mosaic.jpg"])
            # remappings=[('image_in', 'image_raw')]
            ))

    return launch.LaunchDescription(launches)
