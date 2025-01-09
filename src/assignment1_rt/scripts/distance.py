#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class DistanceNode:
    def __init__(self):
        rospy.init_node("distance_node")

        # Parameters
        self.proximity_threshold = 1.0  # Minimum distance between turtles
        self.boundary_limits = [1.0, 10.0]  # x and y boundaries

        # Publishers
        self.distance_pub = rospy.Publisher("/turtle_distance", Float32, queue_size=10)
        self.turtle1_stop_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        self.turtle2_stop_pub = rospy.Publisher("/turtle2/cmd_vel", Twist, queue_size=10)

        # Subscribers
        rospy.Subscriber("/turtle1/pose", Pose, self.turtle1_pose_callback)
        rospy.Subscriber("/turtle2/pose", Pose, self.turtle2_pose_callback)

        # Turtle poses
        self.turtle1_pose = None
        self.turtle2_pose = None

        # States to track if turtles are stopped due to threshold violations
        self.turtle1_stopped = False
        self.turtle2_stopped = False

    def turtle1_pose_callback(self, msg):
        self.turtle1_pose = msg
        self.check_safety("turtle1")

    def turtle2_pose_callback(self, msg):
        self.turtle2_pose = msg
        self.check_safety("turtle2")

    def calculate_distance(self):
        if self.turtle1_pose and self.turtle2_pose:
            dx = self.turtle1_pose.x - self.turtle2_pose.x
            dy = self.turtle1_pose.y - self.turtle2_pose.y
            return (dx ** 2 + dy ** 2) ** 0.5
        return None

    def check_safety(self, turtle_name):
        distance = self.calculate_distance()
        if distance:
            # Publish the distance between turtles
            self.distance_pub.publish(distance)

        # Determine which turtle's pose to check
        pose = self.turtle1_pose if turtle_name == "turtle1" else self.turtle2_pose
        stop_pub = self.turtle1_stop_pub if turtle_name == "turtle1" else self.turtle2_stop_pub

        # Only stop the turtle if it is moving and violates a threshold
        if pose:
            # Check proximity violation
            if distance and distance < self.proximity_threshold:
                if not getattr(self, f"{turtle_name}_stopped"):
                    rospy.loginfo(f"{turtle_name} stopped due to proximity violation.")
                    self.stop_turtle(stop_pub, turtle_name)
                return

            # Check boundary violations
            if pose.x < self.boundary_limits[0] or pose.x > self.boundary_limits[1] or \
               pose.y < self.boundary_limits[0] or pose.y > self.boundary_limits[1]:
                if not getattr(self, f"{turtle_name}_stopped"):
                    rospy.loginfo(f"{turtle_name} stopped due to boundary violation.")
                    self.stop_turtle(stop_pub, turtle_name)

    def stop_turtle(self, stop_pub, turtle_name):
        # Publish zero velocities to stop the turtle
        stop_twist = Twist()
        stop_twist.linear.x = 0
        stop_twist.angular.z = 0
        stop_pub.publish(stop_twist)

        # Mark the turtle as stopped
        setattr(self, f"{turtle_name}_stopped", True)

    def resume_turtle(self, stop_pub, turtle_name):
        # Mark the turtle as stopped, and wait for user input to issue new commands
        setattr(self, f"{turtle_name}_stopped", False)

    def run(self):
        rospy.loginfo("Distance Node is running...")
        rospy.spin()

if __name__ == "__main__":
    try:
        node = DistanceNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Distance Node terminated.")

