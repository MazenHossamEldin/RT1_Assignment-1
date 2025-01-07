#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# Global variables to track stop signals
stop_turtle1 = False
stop_turtle2 = False

def spawn_turtle():
    rospy.wait_for_service('/spawn')
    try:
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        rospy.loginfo("Spawning turtle2...")
        spawn(5.0, 7.0, 0.0, 'turtle2')
        rospy.loginfo("Turtle2 spawned successfully!")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def stop_turtle1_callback(msg):
    global stop_turtle1
    stop_turtle1 = msg.data

def stop_turtle2_callback(msg):
    global stop_turtle2
    stop_turtle2 = msg.data

def main():
    rospy.init_node('ui_node')
    pub1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    pub2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

    rospy.Subscriber('/turtle1/stop', Bool, stop_turtle1_callback)
    rospy.Subscriber('/turtle2/stop', Bool, stop_turtle2_callback)

    rospy.sleep(1)
    spawn_turtle()

    while not rospy.is_shutdown():
        while True:
            turtle_choice = input("Which turtle to control (1 or 2)? ")
            if turtle_choice in ['1', '2']:
                break
            print("Invalid input. Please enter 1 or 2.")

        turtle = "turtle1" if turtle_choice == "1" else "turtle2"
        pub = pub1 if turtle_choice == "1" else pub2
        stop_flag = stop_turtle1 if turtle_choice == "1" else stop_turtle2

        while True:
            try:
                lin_vel = float(input("Enter linear velocity (-10.0 to 10.0): "))
                ang_vel = float(input("Enter angular velocity (-10.0 to 10.0): "))
                if -10.0 <= lin_vel <= 10.0 and -10.0 <= ang_vel <= 10.0:
                    break
                print("Velocity must be between -10.0 and 10.0.")
            except ValueError:
                print("Invalid input. Please enter numeric values.")

        # Allow movement only if the stop flag is not raised
        if stop_flag:
            rospy.loginfo(f"{turtle} is currently stopped. Checking if the command is safe...")
        else:
            twist = Twist()
            twist.linear.x = lin_vel
            twist.angular.z = ang_vel
            rospy.loginfo(f"Publishing to {turtle}: linear.x={lin_vel}, angular.z={ang_vel}")
            pub.publish(twist)

        rospy.sleep(1)

        # Stop the turtle after executing the command
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        pub.publish(twist)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

