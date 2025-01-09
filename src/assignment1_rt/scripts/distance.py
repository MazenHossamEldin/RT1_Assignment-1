#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math

def calculate_distance(pose1, pose2):
    """Calculate Euclidean distance between two poses."""
    return math.sqrt((pose2.x - pose1.x)**2 + (pose2.y - pose1.y)**2)

def boundary_check(pose):
    """Check if a turtle is near or beyond the boundaries of the environment."""
    return pose.x >= 10.0 or pose.x <= 1.0 or pose.y >= 10.0 or pose.y <= 1.0

def distance_node():
    rospy.init_node("distance_node")

    # Publishers to stop turtles if needed
    pub_turtle1 = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    pub_turtle2 = rospy.Publisher("/turtle2/cmd_vel", Twist, queue_size=10)

    # Publisher for distance between turtles
    distance_pub = rospy.Publisher("/distance", Float32, queue_size=10)

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

    rate = rospy.Rate(10)  # 10 Hz loop rate

    threshold_distance = 1.0  # Proximity threshold

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
                rospy.logwarn("Turtle1 is at the boundary! Stopping movement.")
                stop_turtle(pub_turtle1)

            # Enforce boundary thresholds for turtle2
            if boundary_check(turtle2_pose):
                rospy.logwarn("Turtle2 is at the boundary! Stopping movement.")
                stop_turtle(pub_turtle2)

        rate.sleep()

def stop_turtle(publisher):
    """Publish a stop command to the given turtle."""
    stop_cmd = Twist()
    stop_cmd.linear.x = 0
    stop_cmd.angular.z = 0
    publisher.publish(stop_cmd)

if __name__ == "__main__":
    try:
        distance_node()
    except rospy.ROSInterruptException:
        pass

