#!/usr/bin/env python3
# This line specifies that the script should be executed using the Python 3 interpreter.

import rospy
# Import the rospy library for interacting with the ROS system.

from turtlesim.msg import Pose
# Import the Pose message type from the turtlesim package, 
# which represents the position and orientation (x, y, yaw) of a turtle in the simulation.

from std_msgs.msg import Float32
# Import the Float32 message type from the std_msgs package, 
# which represents a single-precision floating-point number.

from geometry_msgs.msg import Twist
# Import the Twist message type from the geometry_msgs package, 
# which represents linear and angular velocities for robot motion.

class DistanceMonitor:
    """
    This class monitors the distance between two turtles in the turtlesim simulation. 
    It publishes the distance and takes actions to prevent collisions and boundary violations.
    """
    def __init__(self):
        """
        Class constructor. 
        Initializes the ROS node, subscribes to turtle pose topics, 
        and creates publishers for distance and velocity commands.
        """
        rospy.init_node('distance_node', anonymous=True) 
        # Initialize a ROS node with the name 'distance_node' 
        # (anonymous=True prevents naming conflicts if multiple instances are running).

        self.turtle1_pose = Pose()  # Store the current pose of turtle1
        self.turtle2_pose = Pose()  # Store the current pose of turtle2

        self.threshold_distance = 3.0  # Minimum distance between turtles before collision avoidance 
        self.boundary_threshold = 3.0  # Distance from the boundary at which to stop turtle1

        rospy.Subscriber('/turtle1/pose', Pose, self.turtle1_callback) 
        # Subscribe to the '/turtle1/pose' topic to receive pose updates for turtle1.
        rospy.Subscriber('/turtle2/pose', Pose, self.turtle2_callback) 
        # Subscribe to the '/turtle2/pose' topic to receive pose updates for turtle2.

        self.distance_pub = rospy.Publisher('/turtle_distance', Float32, queue_size=10) 
        # Create a publisher to publish the distance between turtles on the '/turtle_distance' topic.

        self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) 
        # Create a publisher to publish velocity commands to control turtle1's movement.

    def turtle1_callback(self, data):
        """
        Callback function that updates the stored pose of turtle1 
        when a new pose message is received on the '/turtle1/pose' topic.
        """
        self.turtle1_pose = data

    def turtle2_callback(self, data):
        """
        Callback function that updates the stored pose of turtle2 
        when a new pose message is received on the '/turtle2/pose' topic.
        """
        self.turtle2_pose = data

    def compute_distance(self):
        """
        Calculates the Euclidean distance between the current positions of turtle1 and turtle2.
        """
        dx = self.turtle1_pose.x - self.turtle2_pose.x
        dy = self.turtle1_pose.y - self.turtle2_pose.y
        return (dx**2 + dy**2)**0.5

    def check_boundary_proximity(self):
        """
        Checks if turtle1 is within the boundary_threshold distance of the simulation boundaries.
        """
        if (self.turtle1_pose.x < self.boundary_threshold or 
            self.turtle1_pose.x > 11 - self.boundary_threshold or 
            self.turtle1_pose.y < self.boundary_threshold or 
            self.turtle1_pose.y > 11 - self.boundary_threshold):
            return True  # Turtle1 is too close to the boundary
        return False  # Turtle1 is within the boundaries

    def monitor(self):
        """
        Continuously monitors the distance between turtles and checks for boundary proximity. 
        Publishes the distance and stops turtle1 if necessary.
        """
        rate = rospy.Rate(10)  # Set the loop rate to 10 Hz
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

            rate.sleep()  # Maintain the desired loop rate

if __name__ == '__main__':
    try:
        monitor = DistanceMonitor()
        monitor.monitor()
    except rospy.ROSInterruptException:
        pass
f
