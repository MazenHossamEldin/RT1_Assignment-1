#!/usr/bin/env python3
# This line specifies that the script should be executed using the Python 3 interpreter.

import rospy
# Import the rospy library for interacting with the ROS system.

from turtlesim.srv import Spawn
# Import the Spawn service from the turtlesim package, 
# which allows us to spawn new turtles in the simulation.

from geometry_msgs.msg import Twist
# Import the Twist message type from the geometry_msgs package, 
# which represents linear and angular velocities for robot motion.

def spawn_turtle():
    """
    Spawns a new turtle named 'turtle2' at the specified position (5.0, 5.0) with an initial orientation of 0.0.
    """
    rospy.wait_for_service('/spawn')  # Wait for the '/spawn' service to become available.
    try:
        spawn = rospy.ServiceProxy('/spawn', Spawn)  # Create a proxy to call the '/spawn' service.
        spawn(5.0, 5.0, 0.0, 'turtle2')  # Call the service to spawn the turtle.
        rospy.loginfo("Turtle2 spawned successfully!") 
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")  # Log an error message if the service call fails.

def main():
    """
    Initializes the ROS node, creates publishers for turtle control, 
    spawns a new turtle, and handles user input for controlling turtle movement.
    """
    rospy.init_node('ui_node', anonymous=True)  # Initialize a ROS node with a unique name.
    pub1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=2)  # Create a publisher to send commands to turtle1.
    pub2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=2)  # Create a publisher to send commands to turtle2.

    spawn_turtle()  # Spawn turtle2 at the beginning.

    while not rospy.is_shutdown():  # Continue running until the ROS node is shut down.
        while True:
            turtle_choice = input("Which turtle to control (1 or 2)? ")
            if turtle_choice in ['1', '2']:
                break  # Exit the loop if valid input is provided.
            else:
                print("Invalid input. Please enter 1 or 2.")

        turtle = "turtle1" if turtle_choice == "1" else "turtle2"  # Determine the selected turtle.

        while True:
            try:
                lin_vel_str = input("Enter linear velocity (-10.0 to 10.0): ")
                ang_vel_str = input("Enter angular velocity (-10.0 to 10.0): ")
                lin_vel = float(lin_vel_str)  # Convert user input to float.
                ang_vel = float(ang_vel_str) 
                if -10.0 <= lin_vel <= 10.0 and -10.0 <= ang_vel <= 10.0:
                    break  # Exit the loop if the input is within the valid range.
                else:
                    print("Linear velocity must be between -10.0 and 10.0. Angular velocity must be between -10.0 and 10.0. Try again.")
            except ValueError:
                print("Invalid input. Please enter a valid float number.")

        twist = Twist()  # Create a Twist message to store the desired velocities.
        twist.linear.x = lin_vel  # Set the linear velocity in the x-direction.
        twist.angular.z = ang_vel  # Set the angular velocity around the z-axis.

        if turtle == "turtle1":
            pub1.publish(twist)  # Publish the velocity command to turtle1.
        elif turtle == "turtle2":
            pub2.publish(twist)  # Publish the velocity command to turtle2.

        rospy.sleep(1)  # Wait for 1 second.

        twist.linear.x = 0
        twist.angular.z = 0  # Stop the turtle by setting velocities to zero.
        if turtle == "turtle1":
            pub1.publish(twist)
        elif turtle == "turtle2":
            pub2.publish(twist)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
