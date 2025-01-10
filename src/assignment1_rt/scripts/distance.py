#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
import math

# Store the twist linear velocities for each turtle
turtle1_linear_vel = 0.0
turtle2_linear_vel = 0.0

# Store initial spawn positions for the turtles
turtle1_initial_pose = (5.0, 5.0, 0.0)  # Default spawn position for turtle1
turtle2_initial_pose = (5.0, 8.0, 0.0)  # Custom spawn position for turtle2

def calculate_distance(pose1, pose2):
    """Calculate Euclidean distance between two poses."""
    return math.sqrt((pose2.x - pose1.x)**2 + (pose2.y - pose1.y)**2)

def boundary_check(pose):
    """Check if a turtle is near or beyond the boundaries of the environment."""
    return pose.x >= 10.0 or pose.x <= 1.0 or pose.y >= 10.0 or pose.y <= 1.0

def get_sign(value):
    """Get the sign of a value (-1 for negative, 1 for positive, 0 for zero)."""
    return 1 if value > 0 else -1 if value < 0 else 0

def teleport_turtle(service_proxy, x, y, theta):
    """Teleport a turtle to the specified position."""
    try:
        service_proxy(x, y, theta)
        rospy.loginfo(f"Turtle teleported to: x={x}, y={y}, theta={theta}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to teleport turtle: {e}")

def turtle1_cmd_vel_callback(msg):
    """Callback function to update turtle1's linear velocity."""
    global turtle1_linear_vel
    turtle1_linear_vel = msg.linear.x

def turtle2_cmd_vel_callback(msg):
    """Callback function to update turtle2's linear velocity."""
    global turtle2_linear_vel
    turtle2_linear_vel = msg.linear.x

def distance_node():
    rospy.init_node("distance_node")

    # Publishers to stop turtles if needed
    pub_turtle1 = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)
    pub_turtle2 = rospy.Publisher("/turtle2/cmd_vel", Twist, queue_size=1)

    # Publisher for distance between turtles
    distance_pub = rospy.Publisher("/distance", Float32, queue_size=10)

    # Teleport services for turtles
    teleport_turtle1 = rospy.ServiceProxy("/turtle1/teleport_absolute", TeleportAbsolute)
    teleport_turtle2 = rospy.ServiceProxy("/turtle2/teleport_absolute", TeleportAbsolute)

    # Initialize pose variables
    turtle1_pose = None
    turtle2_pose = None

    def turtle1_pose_callback(msg):
        nonlocal turtle1_pose
        turtle1_pose = msg

    def turtle2_pose_callback(msg):
        nonlocal turtle2_pose
        turtle2_pose = msg

    # Subscribe to pose topics of turtle1 and turtle2
    rospy.Subscriber("/turtle1/pose", Pose, turtle1_pose_callback)
    rospy.Subscriber("/turtle2/pose", Pose, turtle2_pose_callback)

    # Subscribe to cmd_vel topics for turtle1 and turtle2 to get their linear velocities
    rospy.Subscriber("/turtle1/cmd_vel", Twist, turtle1_cmd_vel_callback)
    rospy.Subscriber("/turtle2/cmd_vel", Twist, turtle2_cmd_vel_callback)

    rate = rospy.Rate(10)  # loop rate

    threshold_distance = 2  # Proximity threshold

    while not rospy.is_shutdown():
        if turtle1_pose and turtle2_pose:
            # Calculate the distance between the turtles
            distance = calculate_distance(turtle1_pose, turtle2_pose)
            
            # Publish the distance
            distance_pub.publish(distance)

            # Enforce proximity threshold
            if distance < threshold_distance:
                rospy.logwarn("Turtles are too close! Stopping movement.")
                stop_turtle(pub_turtle1)
                stop_turtle(pub_turtle2)

            # Enforce boundary thresholds for turtle1
            if boundary_check(turtle1_pose):
                rospy.logwarn("Turtle1 is at the boundary! Teleporting to initial position.")
                teleport_turtle(teleport_turtle1, *turtle1_initial_pose)

            # Enforce boundary thresholds for turtle2
            if boundary_check(turtle2_pose):
                rospy.logwarn("Turtle2 is at the boundary! Teleporting to initial position.")
                teleport_turtle(teleport_turtle2, *turtle2_initial_pose)

            # Log linear velocities for debugging
            rospy.loginfo(f"Turtle1 linear velocity: {turtle1_linear_vel}")
            rospy.loginfo(f"Turtle2 linear velocity: {turtle2_linear_vel}")

        rate.sleep()

def stop_turtle(publisher):
    """Stop the turtle and move it slightly backward based on the direction of its velocity."""
    rospy.loginfo("Stopping a turtle at the threshold.")
    stop_cmd = Twist()
    stop_cmd.linear.x = 0
    stop_cmd.angular.z = 0
    publisher.publish(stop_cmd)
    rospy.sleep(1)

    # Use the sign of the velocity to move backward
    if publisher.resolved_name == "/turtle1/cmd_vel":
        linear_velocity_sign = get_sign(turtle1_linear_vel)
    elif publisher.resolved_name == "/turtle2/cmd_vel":
        linear_velocity_sign = get_sign(turtle2_linear_vel)
    else:
        linear_velocity_sign = 0  # Default to 0 if no valid publisher is matched

    stop_cmd.linear.x = -2 * linear_velocity_sign
    publisher.publish(stop_cmd)
    rospy.sleep(1)  # Adjust duration as needed

    #rospy.loginfo("Turtle stopped after moving away.")
    #stop_cmd.linear.x = 0
    #stop_cmd.angular.z = 0
    #publisher.publish(stop_cmd)

if __name__ == "__main__":
    try:
        distance_node()
    except rospy.ROSInterruptException:
        pass

