# v4l2ucp

ROS wrapper for all Video for Linux Two (V4L2) devices controls.

    git clone -b ros2 https://github.com/lucasw/v4l2ucp.git
    
Then move to the ros2 ws directory:

    colcon build --packages-select v4l2ucp
    ros2 run v4l2ucp v4l2ucp  # will default to device 0
