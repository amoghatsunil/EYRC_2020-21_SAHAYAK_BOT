#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time


# when node is subscribed to topic,every time message arrives on it,
# the associated callback function is called
def pose_callback(pose_message):
    global x, y, yaw
    x = pose_message.x  # x value of position
    y = pose_message.y  # y value of position
    yaw = pose_message.theta  # theta value of position


# this is the function to move turlte in a circle
def move_circle():
    # declaring velocity publisher
    vel_topic = "/turtle1/cmd_vel"  # topic name

    # advertising the node by publishing it
    pub = rospy.Publisher(vel_topic, Twist, queue_size=10)

    vel = Twist()  # twist message to send velocity commands
    lin_vel = 1
    ang_vel = 1
    vel.linear.x = lin_vel  # linear velocity in x
    vel.angular.z = ang_vel  # angular velocity in z
    rate = rospy.Rate(5)  # publish velocity at 10Hz (10 times)
    pi = 3.141592653589793238
    r = 1  # radius is given by linear velocity/angular velocity
    circum = 2 * pi * r  # circumference
    t0 = rospy.Time.now().to_sec()  # storing initial time
    dist = 1  # initializing distance travelled by the turtle to 0

    while not rospy.is_shutdown():
        # The is_shutdown() function will return True if the
        #  node is ready to be shut down and False otherwise
        rospy.loginfo("Circular motion")
        pub.publish(vel)  # publishing our velocity message
        t1 = rospy.Time.now().to_sec()  # time at each interval
        dist = (t1 - t0) * 1  # calculating distance travelled by the turtle
        print dist  # printing the distance travelled
        rate.sleep()

        if dist >= (circum):
            # checking if distance travelled by the turtle is greater than
            # or equal to circumference(i.e one complete circle)
            rospy.loginfo("Destination Reached")
            break

    # stop the turtle when it completes one circle
    vel.linear.x = 0
    vel.angular.z = 0
    pub.publish(vel)


# this function is used to rotate the turtle after it completes one circle
def rotate(ang_vel_deg, req_angle):
    # declaring velocity publisher
    vel_topic = "/turtle1/cmd_vel"  # topic name
    # advertising the node by publishing it
    pub = rospy.Publisher(vel_topic, Twist, queue_size=10)

    vel = Twist()  # twist message to send velocity commands
    ang_vel = math.radians(abs(ang_vel_deg))
    # converting the given angle to radians

    vel.angular.z = abs(ang_vel)
    # setting value of angular velocity in z to required angle
    rate = rospy.Rate(5)  # publish velocity at 10Hz (10 times)
    t0 = rospy.Time.now().to_sec()  # storing initial time
    curr_ang = 0  # intialize angle made by turtle to zero

    while not rospy.is_shutdown():
        # The is_shutdown() function will return True if the node
        # is ready to be shut down and False otherwise
        pub.publish(vel)  # publishing our velocity message
        t1 = rospy.Time.now().to_sec()
        curr_ang = (t1 - t0) * ang_vel_deg
        # calculating the angle at each iteration
        rate.sleep()

        if curr_ang > req_angle:
            # checking if the angle made by turtle is equal or
            # greater than required angle(to stop)
            break

    vel.linear.z = 0  # stop the turtle when it rotates to required angle
    pub.publish(vel)  # publishing velocity message


# main function
if __name__ == "__main__":
    try:
        # initialize a node
        rospy.init_node("node_turtle_revolve", anonymous=True)

        # velocity subscirber
        position_topic = "/turtle1/pose"  # topic name
        # subscribing to position message
        velocity_subscriber = rospy.Subscriber(position_topic,
                                               Pose, pose_callback)
        time.sleep(1)

        move_circle()  # calling move_circle function
        rotate(50, 15)  # calling rotate function

    except rospy.ROSInterruptException:  # if exception is thrown
        rospy.loginfo("Node terminated")
