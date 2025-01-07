#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class DistanceMonitor:
    def __init__(self):
        rospy.init_node('distance_node', anonymous=True)

        self.turtle1_pose = Pose()
        self.turtle2_pose = Pose()
        self.threshold_distance = 4.0  # Distance threshold for proximity warning
        self.boundary_threshold = 2.0  # Distance from edges of the Turtlesim grid

        # Subscribers to pose topics for both turtles
        rospy.Subscriber('/turtle1/pose', Pose, self.turtle1_callback)
        rospy.Subscriber('/turtle2/pose', Pose, self.turtle2_callback)

        # Publishers for velocity commands to both turtles
        self.cmd_vel_pub_t1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.cmd_vel_pub_t2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

        # Publisher to publish the distance between the turtles
        self.distance_pub = rospy.Publisher('/turtle_distance', Float32, queue_size=10)

    def turtle1_callback(self, data):
        self.turtle1_pose = data

    def turtle2_callback(self, data):
        self.turtle2_pose = data

    def compute_distance(self):
        dx = self.turtle1_pose.x - self.turtle2_pose.x
        dy = self.turtle1_pose.y - self.turtle2_pose.y
        return (dx**2 + dy**2)**0.5

    def check_boundary_proximity(self, pose):
        """
        Check if a turtle is close to the boundary.
        """
        return (
            pose.x <= self.boundary_threshold or
            pose.x >= 11.0 - self.boundary_threshold or
            pose.y <= self.boundary_threshold or
            pose.y >= 11.0 - self.boundary_threshold
        )

    def stop_turtles(self):
        """
        Stop both turtles by publishing zero velocity commands.
        """
        stop_cmd = Twist()  # Zero velocity for stopping
        self.cmd_vel_pub_t1.publish(stop_cmd)
        self.cmd_vel_pub_t2.publish(stop_cmd)

    def monitor(self):
        rate = rospy.Rate(10)  # Monitor at 10 Hz
        while not rospy.is_shutdown():
            # Compute the distance between turtles
            distance = self.compute_distance()
            self.distance_pub.publish(distance)

            # Check if either turtle is near the other or close to the boundary
            if distance < self.threshold_distance:
                rospy.loginfo("Turtles are too close to each other! Stopping both.")
                self.stop_turtles()
            elif self.check_boundary_proximity(self.turtle1_pose):
                rospy.loginfo("Turtle1 is near the boundary! Stopping both.")
                self.stop_turtles()
            elif self.check_boundary_proximity(self.turtle2_pose):
                rospy.loginfo("Turtle2 is near the boundary! Stopping both.")
                self.stop_turtles()

            rate.sleep()

if __name__ == '__main__':
    try:
        monitor = DistanceMonitor()
        monitor.monitor()
    except rospy.ROSInterruptException:
        pass

