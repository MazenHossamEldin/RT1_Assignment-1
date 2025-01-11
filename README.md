# Turtlesim Distance Monitoring Package

This ROS package is designed to control two turtles in the Turtlesim environment, monitor the distance between them, and enforce boundary checks and proximity thresholds. The package consists of two main nodes: `distance_node` and `ui_node`.

## Project Description

This project aims to demonstrate basic ROS concepts such as creating and using nodes, publishing and subscribing to topics, and using services. The `distance_node` monitors the distance between two turtles and ensures they do not get too close or move out of bounds. The `ui_node` provides a simple user interface to control the turtles' velocities.

## Nodes

### distance_node

The `distance_node` is responsible for:
- Calculating the distance between two turtles.
- Publishing the distance between the turtles on the `/distance` topic.
- Stopping the turtles if they get too close to each other.
- Teleporting the turtles back to their initial positions if they move out of bounds.
- Logging the linear velocities of the turtles for debugging purposes.

#### Subscribed Topics
- `/turtle1/pose` (Pose): Receives the pose of turtle1.
- `/turtle2/pose` (Pose): Receives the pose of turtle2.
- `/turtle1/cmd_vel` (Twist): Receives the linear velocity of turtle1.
- `/turtle2/cmd_vel` (Twist): Receives the linear velocity of turtle2.

#### Published Topics
- `/distance` (Float32): Publishes the distance between the two turtles.
- `/turtle1/cmd_vel` (Twist): Publishes commands to control turtle1.
- `/turtle2/cmd_vel` (Twist): Publishes commands to control turtle2.

### ui_node

The `ui_node` provides a user interface to control the velocities of the turtles. It allows the user to select a turtle and input linear and angular velocities, which are then published to the respective turtles' cmd_vel topics.

#### Subscribed Topics
- `/teleport_notification` (String): Receives teleport notifications from the distance node.

#### Published Topics
- `/turtle1/cmd_vel` (Twist): Publishes commands to control turtle1.
- `/turtle2/cmd_vel` (Twist): Publishes commands to control turtle2.

## Installation

1. **Install ROS**: Follow the instructions on the [ROS installation page](http://wiki.ros.org/ROS/Installation) to install ROS (recommended version: Noetic).

2. **Create a Catkin Workspace**:
    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make
    source devel/setup.bash
    ```

3. **Clone the Package**:
    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/MazenHossamEldin/RT1_Assignment-1.git
    cd ..
    catkin_make
    source devel/setup.bash
    ```

## Running the Nodes

1. **Start the Turtlesim Node**:
    ```bash
    roscore
    ```

    Open a new terminal and run:
    ```bash
    rosrun turtlesim turtlesim_node
    ```

2. **Run the Distance Node**:
    Open a new terminal and run:
    ```bash
    rosrun assignment1_rt distance.py
    ```

3. **Run the UI Node**:
    Open a new terminal and run:
    ```bash
    rosrun assignment1_rt ui.py
    ```

Now you can use the UI node to control the turtles and observe the distance monitoring functionality in action. The turtles will be teleported back to their initial positions if they move out of bounds, and their movements will be stopped if they get too close to each other.
