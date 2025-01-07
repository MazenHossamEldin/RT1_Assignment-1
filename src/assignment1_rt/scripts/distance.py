#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from std_msgs.msg import Float32
from rospy.core import rospy  # Import rospy.loginfo

class DistanceMonitor:
    """
    This class defines a DistanceMonitor object that monitors the distance between two turtles in a ROS simulation.
    """

    def __init__(self):
        """
        This is the constructor of the DistanceMonitor class. It is called when an instance of the class is created.
        """
        rospy.init_node('distance_node', anonymous=True)

        self.turtle1_pose = Pose()
        self.turtle2_pose = Pose()
        self.threshold = 1.0

        rospy.Subscriber('/turtle1/pose', Pose, self.turtle1_callback)
        rospy.Subscriber('/turtle2/pose', Pose, self.turtle2_callback)

        self.pub = rospy.Publisher('/turtle_distance', Float32, queue_size=10)

    def turtle1_callback(self, data):
        """
        This function is called whenever a new message is received on the '/turtle1/pose' topic.
        """
        self.turtle1_pose = data

    def turtle2_callback(self, data):
        """
        This function is called whenever a new message is received on the '/turtle2/pose' topic.
        """
        self.turtle2_pose = data

    def compute_distance(self):
        """
        This function calculates the Euclidean distance between the poses of turtle1 and turtle2.
        """
        dx = self.turtle1_pose.x - self.turtle2_pose.x
        dy = self.turtle1_pose.y - self.turtle2_pose.y
        return (dx**2 + dy**2)**0.5

    def monitor(self):
        """
        This function continuously monitors the distance between the turtles and publishes the distance and warning messages if necessary.
        """
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            distance = self.compute_distance()
            self.pub.publish(distance)

            if distance < self.threshold:
                rospy.loginfo("Distance between turtles: %s", distance)  # Fixed indentation and closing parenthesis

            rate.sleep()

if __name__ == '__main__':
    try:
        monitor = DistanceMonitor()
        monitor.monitor()
    except rospy.ROSInterruptException:
        pass
