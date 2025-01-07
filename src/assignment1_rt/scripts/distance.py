#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from std_msgs.msg import Float32, Bool

class DistanceMonitor:
    def __init__(self):
        rospy.init_node('distance_node')

        self.turtle1_pose = Pose()
        self.turtle2_pose = Pose()
        self.proximity_threshold = 2.0  # Distance threshold for proximity
        self.boundary_threshold = 1.0  # Distance from edges of the Turtlesim grid

        # Subscribers to pose topics for both turtles
        rospy.Subscriber('/turtle1/pose', Pose, self.turtle1_callback)
        rospy.Subscriber('/turtle2/pose', Pose, self.turtle2_callback)

        # Publishers for distance and stop signals
        self.distance_pub = rospy.Publisher('/turtle_distance', Float32, queue_size=10)
        self.stop_pub_t1 = rospy.Publisher('/turtle1/stop', Bool, queue_size=10)
        self.stop_pub_t2 = rospy.Publisher('/turtle2/stop', Bool, queue_size=10)

    def turtle1_callback(self, data):
        self.turtle1_pose = data

    def turtle2_callback(self, data):
        self.turtle2_pose = data

    def compute_distance(self):
        dx = self.turtle1_pose.x - self.turtle2_pose.x
        dy = self.turtle1_pose.y - self.turtle2_pose.y
        return (dx**2 + dy**2)**0.5

    def check_boundary_proximity(self, pose):
        return (
            pose.x <= self.boundary_threshold or
            pose.x >= 11.0 - self.boundary_threshold or
            pose.y <= self.boundary_threshold or
            pose.y >= 11.0 - self.boundary_threshold
        )

    def monitor(self):
        rate = rospy.Rate(10)  # Monitor at 10 Hz
        while not rospy.is_shutdown():
            try:
                # Compute the distance between turtles
                distance = self.compute_distance()
                self.distance_pub.publish(distance)

                # Check proximity and boundary conditions for both turtles
                stop_t1 = self.check_boundary_proximity(self.turtle1_pose) or (distance < self.proximity_threshold)
                stop_t2 = self.check_boundary_proximity(self.turtle2_pose) or (distance < self.proximity_threshold)

                # Publish stop signals for both turtles
                self.stop_pub_t1.publish(stop_t1)
                self.stop_pub_t2.publish(stop_t2)

            except Exception as e:
                rospy.logerr(f"An error occurred: {e}")

            rate.sleep()

if __name__ == '__main__':
    try:
        monitor = DistanceMonitor()
        monitor.monitor()
    except rospy.ROSInterruptException:
        pass

