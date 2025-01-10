# Turtlesim Control Package

This package provides a simulation framework to control and monitor two turtles within the ROS `turtlesim` environment. It includes functionality to calculate the distance between two turtles, enforce proximity and boundary thresholds, and allow user interaction via a UI for controlling the turtles' movement.

## Overview of the Project

The project simulates two turtles in the `turtlesim` environment. The following functionalities are provided:

1. **Distance Calculation and Threshold Enforcement**:
    - A node calculates the Euclidean distance between two turtles and enforces a minimum proximity threshold.
    - The turtles are stopped if they come too close to each other or approach the boundary of the environment.

2. **User Interaction via a UI**:
    - A user interface node allows the user to control the turtles' movements by specifying linear and angular velocities.

## Nodes

### 1. `distance.py`
This node monitors the turtles' positions, calculates the distance between them, and enforces safety thresholds. It performs the following tasks:
- Calculates the Euclidean distance between `turtle1` and `turtle2`.
- Stops a turtle if it:
  - Comes too close to the other turtle (distance < 1.5 units).
  - Reaches the boundary of the simulation environment.
- Publishes the calculated distance on the `/distance` topic.

#### Key Topics:
- **Subscribed Topics**:
  - `/turtle1/pose`: Retrieves the position of `turtle1`.
  - `/turtle2/pose`: Retrieves the position of `turtle2`.
  - `/turtle1/cmd_vel`: Monitors `turtle1`'s velocity.
  - `/turtle2/cmd_vel`: Monitors `turtle2`'s velocity.
- **Published Topics**:
  - `/turtle1/cmd_vel`: Sends stop or backward movement commands to `turtle1`.
  - `/turtle2/cmd_vel`: Sends stop or backward movement commands to `turtle2`.
  - `/distance`: Publishes the distance between the two turtles.

#### Thresholds:
- **Proximity Threshold**: Stops turtles if they are closer than 1.5 units.
- **Boundary Threshold**: Stops turtles if they reach or exceed the simulation boundaries (x or y in `[1.0, 10.0]`).

### 2. `UI.py`
This node provides a user-friendly interface to control the turtles by specifying their linear and angular velocities.

#### Key Features:
- Spawns `turtle2` at the position (5.0, 8.0).
- Allows the user to:
  - Select which turtle to control (`turtle1` or `turtle2`).
  - Input linear and angular velocities for the selected turtle.
- Sends velocity commands to the turtles and automatically stops them after 1 second.

#### Key Topics:
- **Published Topics**:
  - `/turtle1/cmd_vel`: Sends velocity commands to `turtle1`.
  - `/turtle2/cmd_vel`: Sends velocity commands to `turtle2`.

## Installation and Running the Package

### Prerequisites
- ROS installed on your system (tested with ROS Noetic).
- `turtlesim` package installed.

### Installation Steps
1. Clone the package into your ROS workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone <repository-link>
