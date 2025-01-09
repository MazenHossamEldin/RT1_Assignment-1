#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn

def main():
    rospy.init_node("ui_node")

    # Publishers for turtle commands
    pub_turtle1 = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    pub_turtle2 = rospy.Publisher("/turtle2/cmd_vel", Twist, queue_size=10)

    # Spawn turtle2
    rospy.wait_for_service("/spawn")
    try:
        spawn_turtle = rospy.ServiceProxy("/spawn", Spawn)
        spawn_turtle(5.0, 5.0, 0.0, "turtle2")
        rospy.loginfo("Spawned turtle2 at (5.0, 5.0)")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to spawn turtle2: {e}")
        return

    while not rospy.is_shutdown():
        try:
            # Turtle selection
            turtle_choice = input("Select a turtle to control:\n1. turtle1\n2. turtle2\nEnter your choice (1 or 2): ").strip()
            if turtle_choice not in ["1", "2"]:
                print("Invalid input. Please choose 1 (turtle1) or 2 (turtle2).")
                continue

            # Velocity input
            try:
                linear_vel = float(input("Enter linear velocity (float): ").strip())
                angular_vel = float(input("Enter angular velocity (float): ").strip())
            except ValueError:
                print("Invalid velocity. Please enter numeric values for both linear and angular velocities.")
                continue

            # Feedback for user input
            turtle_name = "turtle1" if turtle_choice == "1" else "turtle2"
            print(f"Controlling {turtle_name} with linear velocity {linear_vel} and angular velocity {angular_vel}.")

            # Create and publish velocity command
            twist = Twist()
            twist.linear.x = linear_vel
            twist.angular.z = angular_vel
            if turtle_choice == "1":
                pub_turtle1.publish(twist)
            else:
                pub_turtle2.publish(twist)

            # Hold the command for 1 second
            rospy.sleep(1.0)

            # Stop the turtle
            twist.linear.x = 0
            twist.angular.z = 0
            if turtle_choice == "1":
                pub_turtle1.publish(twist)
            else:
                pub_turtle2.publish(twist)

            print("Command executed. Turtle has stopped. Enter a new command.")

        except KeyboardInterrupt:
            print("\nExiting UI node. Goodbye!")
            break

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

