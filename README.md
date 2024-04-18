# TurtleSim fractal tree drawer
The project was developed as a semester project for the ROS2 course at Óbuda University

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

How to set *parameter*.

The node has only one parameter which sets how big of a tree we want to draw. 
The value must be in range [0.5; 2.5] to ensure proper behaviour. 

    ros2 param set turtlesim_controller iter <your_float_value>

