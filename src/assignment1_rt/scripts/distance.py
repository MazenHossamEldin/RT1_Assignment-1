#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rospy.core import rospy

class DistanceMonitor:
    def __init__(self):
        rospy.init_node('distance_node', anonymous=True)

        self.turtle1_pose = Pose()
        self.turtle2_pose = Pose()
        self.threshold_distance = 1.0  # Distance threshold for proximity warning
        self.boundary_threshold = 1.0  # Distance threshold for boundary proximity

        rospy.Subscriber('/turtle1/pose', Pose, self.turtle1_callback)
        rospy.Subscriber('/turtle2/pose', Pose, self.turtle2_callback)
        self.distance_pub = rospy.Publisher('/turtle_distance', Float32, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)  # Publish to turtle1's velocity

    def turtle1_callback(self, data):
        self.turtle1_pose = data

    def turtle2_callback(self, data):
        self.turtle2_pose = data

    def compute_distance(self):
        dx = self.turtle1_pose.x - self.turtle2_pose.x
        dy = self.turtle1_pose.y - self.turtle2_pose.y
        return (dx**2 + dy**2)**0.5

    def check_boundary_proximity(self):
        if (self.turtle1_pose.x < self.boundary_threshold or 
            self.turtle1_pose.x > 11 - self.boundary_threshold or 
            self.turtle1_pose.y < self.boundary_threshold or 
            self.turtle1_pose.y > 11 - self.boundary_threshold):
            return True
        return False

    def monitor(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            distance = self.compute_distance()
            self.distance_pub.publish(distance)

            if distance < self.threshold_distance:
                rospy.loginfo("Turtles are too close!")
                # Stop turtle1 if it's too close to turtle2
                cmd_vel_msg = Twist()
                self.cmd_vel_pub.publish(cmd_vel_msg) 

            if self.check_boundary_proximity():
                rospy.loginfo("Turtle1 is too close to the boundary!")
                # Stop turtle1 if it's too close to the boundary
                cmd_vel_msg = Twist()
                self.cmd_vel_pub.publish(cmd_vel_msg) 

            rate.sleep()

if __name__ == '__main__':
    try:
        monitor = DistanceMonitor()
        monitor.monitor()
    except rospy.ROSInterruptException:
        pass
