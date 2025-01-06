#!/usr/bin/env python3
# This line ensures that the script runs with the Python 3 interpreter.

import rospy  # Importing the rospy package, which is used to interact with ROS.
from turtlesim.srv import Spawn  # Importing the Spawn service from the turtlesim package.
from geometry_msgs.msg import Twist  # Importing the Twist message type, which contains information about velocities.

# Function to spawn a new turtle (turtle2).
def spawn_turtle():
    # Wait until the /spawn service is available.
    rospy.wait_for_service('/spawn')
    try:
        # Creating a proxy to call the /spawn service.
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        # Calling the spawn service to spawn a new turtle at position (5.0, 5.0) with an orientation of 0.0 and name 'turtle2'.
        spawn(5.0, 5.0, 0.0, 'turtle2')
        # Logging info message when the turtle is spawned successfully.
        rospy.loginfo("Turtle2 spawned successfully!")
    except rospy.ServiceException as e:
        # If the service call fails, an error message is logged.
        rospy.logerr(f"Service call failed: {e}")

# Main function to control turtles.
def main():
    # Initialize the ROS node with the name 'ui_node' and ensure the name is unique (anonymous=True).
    rospy.init_node('ui_node', anonymous=True)

    # Creating two publishers to control turtle1 and turtle2's velocities. The /turtle1/cmd_vel and /turtle2/cmd_vel topics are used for controlling the turtles.
    pub1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    pub2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

    # Spawning turtle2 at the specified position.
    spawn_turtle()

    # Loop that runs until ROS is shut down.
    while not rospy.is_shutdown():
        # Asking the user which turtle they want to control.
        turtle = input("Which turtle to control (turtle1/turtle2)? ")
        # Asking the user for the velocity of the turtle.
        vel = float(input("Enter velocity (linear, 0-2): "))

        # Creating a Twist message to store the linear velocity (angular velocity is not set here).
        twist = Twist()
        twist.linear.x = vel  # Setting the linear velocity in the x-direction.

        # Checking which turtle the user wants to control and publishing the velocity command.
        if turtle == 'turtle1':
            pub1.publish(twist)
        elif turtle == 'turtle2':
            pub2.publish(twist)
        else:
            # If the user enters an invalid turtle name, a warning is logged.
            rospy.logwarn("Invalid turtle name!")

        # Sleeping for 1 second to give time for the movement to take effect.
        rospy.sleep(1)

        # After the sleep, setting the velocity to 0 to stop the turtle.
        twist.linear.x = 0
        if turtle == 'turtle1':
            pub1.publish(twist)
        elif turtle == 'turtle2':
            pub2.publish(twist)

# The script entry point.
if __name__ == '__main__':
    try:
        # Running the main function.
        main()
    except rospy.ROSInterruptException:
        # Catching the exception if the ROS node is interrupted.
        pass

