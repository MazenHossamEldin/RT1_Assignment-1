#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist

def spawn_turtle():
    rospy.wait_for_service('/spawn')
    try:
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        rospy.loginfo("Spawning turtle2...")
        spawn(5.0, 5.0, 0.0, 'turtle2')
        rospy.loginfo("Turtle2 spawned successfully!")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def main():
    rospy.init_node('ui_node', anonymous=True)
    pub1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    pub2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

    rospy.sleep(1)  # Ensure topics are initialized
    spawn_turtle()

    while not rospy.is_shutdown():
        while True:
            turtle_choice = input("Which turtle to control (1 or 2)? ")
            if turtle_choice in ['1', '2']:
                break
            else:
                print("Invalid input. Please enter 1 or 2.")

        turtle = "turtle1" if turtle_choice == "1" else "turtle2"

        while True:
            try:
                lin_vel = float(input("Enter linear velocity (-10.0 to 10.0): "))
                ang_vel = float(input("Enter angular velocity (-10.0 to 10.0): "))
                if -10.0 <= lin_vel <= 10.0 and -10.0 <= ang_vel <= 10.0:
                    break
                else:
                    print("Velocity must be between -10.0 and 10.0. Try again.")
            except ValueError:
                print("Invalid input. Please enter numeric values.")

        twist = Twist()
        twist.linear.x = lin_vel
        twist.angular.z = ang_vel

        rospy.loginfo(f"Publishing to {turtle}: linear.x={lin_vel}, angular.z={ang_vel}")
        if turtle == "turtle1":
            pub1.publish(twist)
        elif turtle == "turtle2":
            pub2.publish(twist)

        rospy.sleep(1) 

        # Stop the turtle after movement
        twist.linear.x = 0
        twist.angular.z = 0
        rospy.loginfo(f"Stopping {turtle}")
        if turtle == "turtle1":
            pub1.publish(twist)
        elif turtle == "turtle2":
            pub2.publish(twist)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

