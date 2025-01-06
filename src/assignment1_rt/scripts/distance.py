#!/usr/bin/env python3
# This line specifies the interpreter to be used for executing this script. In this case, it's the Python 3 interpreter.

import rospy
# This line imports the rospy library, which provides functionalities for working with ROS (Robot Operating System).

from turtlesim.msg import Pose
# This line imports the Pose message class from the turtlesim package. This message class is used to represent the pose (position and orientation) of a turtle in the simulation.

from std_msgs.msg import Float32
# This line imports the Float32 message class from the std_msgs package. This message class is used to represent a single-precision floating-point value.

class DistanceMonitor:
  """
  This class defines a DistanceMonitor object that monitors the distance between two turtles in a ROS simulation.
  """
  def __init__(self):
    """
    This is the constructor of the DistanceMonitor class. It is called when an instance of the class is created.
    """
    rospy.init_node('distance_node', anonymous=True)
    # Initializes a ROS node named 'distance_node'. The anonymous argument is set to True to avoid naming conflicts if multiple instances of this node are running simultaneously.

    self.turtle1_pose = Pose()
    # Creates an instance of the Pose message class to store the pose of turtle1.

    self.turtle2_pose = Pose()
    # Creates an instance of the Pose message class to store the pose of turtle2.

    self.threshold = 1.0
    # Defines a threshold distance (1.0 meters in this case) for triggering a warning message.

    rospy.Subscriber('/turtle1/pose', Pose, self.turtle1_callback)
    # Subscribes to the ROS topic '/turtle1/pose' that publishes messages of type Pose. The self.turtle1_callback function will be called whenever a new message is received on this topic.

    rospy.Subscriber('/turtle2/pose', Pose, self.turtle2_callback)
    # Subscribes to the ROS topic '/turtle2/pose' that publishes messages of type Pose. The self.turtle2_callback function will be called whenever a new message is received on this topic.

    self.pub = rospy.Publisher('/turtle_distance', Float32, queue_size=10)
    # Creates a ROS publisher named 'pub' that publishes messages of type Float32 on the topic '/turtle_distance'. The queue_size argument specifies the maximum number of messages to be buffered in case the subscriber cannot keep up with the publisher's rate.

  def turtle1_callback(self, data):
    """
    This function is called whenever a new message is received on the '/turtle1/pose' topic.
    """
    self.turtle1_pose = data
    # Updates the self.turtle1_pose attribute with the received Pose data.

  def turtle2_callback(self, data):
    """
    This function is called whenever a new message is received on the '/turtle2/pose' topic.
    """
    self.turtle2_pose = data
    # Updates the self.turtle2_pose attribute with the received Pose data.

  def compute_distance(self):
    """
    This function calculates the Euclidean distance between the poses of turtle1 and turtle2.
    """
    dx = self.turtle1_pose.x - self.turtle2_pose.x
    # Calculates the difference in x-coordinates between the two turtles.

    dy = self.turtle1_pose.y - self.turtle2_pose.y
    # Calculates the difference in y-coordinates between the two turtles.

    return (dx**2 + dy**2)**0.5
    # Applies the Pythagorean theorem to compute the distance based on the difference in x and y coordinates.

  def monitor(self):
    """
    This function continuously monitors the distance between the turtles and publishes the distance and warning messages if necessary.
    """
    rate = rospy.Rate(10)
    # Creates a ROS rate object to control the loop rate at 10 Hz (10 cycles per second).

    while not rospy.is_shutdown():
      # This loop continues until ROS is shut down.

      distance = self.compute_distance()
      # Calls the compute_distance function to calculate the current distance between the turtles.

      self.pub.publish(distance)
      # Publishes the calculated distance on the '/turtle_distance' topic.

      if distance < self.threshold:
        rospy
