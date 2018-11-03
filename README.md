# v4l2ucp

ROS wrapper for all Video for Linux Two (V4L2) devices controls.

v4l2ucp - A universal control panel for all Video for Linux Two (V4L2) devices.

This is port of an original v4l2ucp graphical V4L configuration tool to a
non-graphical ros node. Original version was written Scott J. Bertin.

This software is written in C++ using ROS2 libraries on Linux. It reads a
description of the controls that the V4L2 device supports from the device,
and presents the user with ros2 service for adjusting those controls.
It allows for controlling multiple devices. There is an easy way
to reset one or all the controls to their default state.

It will try to open /dev/video0 if nothing else was specified.
If no devices can be opened, the program will exit.

## Instructions

    git clone -b ros2 https://github.com/lucasw/v4l2ucp.git

Then move to the ros2 ws directory:

    colcon build --packages-select v4l2ucp
    ros2 run v4l2ucp v4l2ucp  # will default to device 0

If usb_cam (https://github.com/lucasw/usb_cam/tree/ros2) is built use the launch instead of run:

    ros2 launch v4l2ucp v4l2ucp_launch.py

Then there are topics and services available to change parameters
    $ ros2 topic list
    /clock
    /controls/BacklightCompensation
    /controls/Brightness
    /controls/Contrast
    /controls/Gamma
    /controls/Hue
    /controls/PowerLineFrequency
    /controls/Saturation
    /controls/Sharpness
    /controls/WhiteBalanceTemperature
    /controls/WhiteBalanceTemperatureAuto
    /feedback/BacklightCompensation
    /feedback/Brightness
    /feedback/Contrast
    /feedback/Gamma
    /feedback/Hue
    /feedback/PowerLineFrequency
    /feedback/Saturation
    /feedback/Sharpness
    /feedback/WhiteBalanceTemperature
    /feedback/WhiteBalanceTemperatureAuto
    /parameter_events

    ros2 topic pub /controls/Contrast std_msgs/Int32 "{data: 150}"

    $ ros2 service list
    /get_control
    /set_control
    /v4l2ucp/describe_parameters
    /v4l2ucp/get_parameter_types
    /v4l2ucp/get_parameters
    /v4l2ucp/list_parameters
    /v4l2ucp/set_parameters
    /v4l2ucp/set_parameters_atomically

Query a control:

    $ ros2 service call /get_control v4l2ucp/SetControl "{control: {name: Gamma}}"
    requester: making request: v4l2ucp.srv.SetControl_Request(control=v4l2ucp.msg.Control(name='Gamma', type=0, value=0, min=0, max=1))

    response:
    v4l2ucp.srv.SetControl_Response(success=False, message='', control=v4l2ucp.msg.Control(name='Gamma', type=1, value=300, min=100, max=300))

Set a control:

    $ ros2 service call /set_control v4l2ucp/SetControl "{control: {name: Gamma, value: 250}}"
    requester: making request: v4l2ucp.srv.SetControl_Request(control=v4l2ucp.msg.Control(name='Gamma', type=0, value=250, min=0, max=1))

    response:
    v4l2ucp.srv.SetControl_Response(success=False, message='', control=v4l2ucp.msg.Control(name='Gamma', type=1, value=250, min=100, max=300))
