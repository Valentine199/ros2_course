# TurtleSim fractal tree drawer

## About

The package draws a fractal tree in the ROS2 turtlesim tool. 
To achive this it uses a turtlesim turtle which is capable of turning and going on a straight line. For these movements the turtle uses a P-controller.

## Usage

How to *build* the package.

    cd ~/ros2_ws
    colcon build --symlink-install

How to *run* the package.

    cd ~/ros2_ws
    ros2 launch ros2_course turtlesim_launch.py

