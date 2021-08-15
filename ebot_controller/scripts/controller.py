#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
from geometry_msgs.msg import Point
pose=[0,0,0]
regions = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft':0,
        'left':  0
    }
x_goal=12.5
y_goal=0

def Waypoints(x):
    x2 =0.3+x
    y2 = 2*math.sin(x2)*math.sin(x2/2)
    return [x2,y2]

def control_loop():
    global pose
    global regions
    rospy.init_node('ebot_controller')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.Subscriber('/ebot/laser/scan',LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rate = rospy.Rate(10) 
    
    P = 2
    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)
    
    while not rospy.is_shutdown():
        if regions==None:
            continue
        x2,y2=Waypoints(pose[0])
        dy = y2-pose[1]
        dx = x2-pose[0]
        goal =math.atan(dy/dx)
        e_theta=goal-pose[2]

        velocity_msg.linear.x = 0.3
        velocity_msg.angular.z = P*e_theta
        distance=math.sqrt((pose[0]-12.5)**2+(pose[1])**2)
        pub.publish(velocity_msg)
        
        if(distance<=6.2):
            rospy.loginfo("Covered waypoints")
            break

    velocity_msg.linear.x=0
    velocity_msg.angular.z=0
    pub.publish(velocity_msg) 
    rospy.sleep(0.1)
    d=2.2
    
    while not rospy.is_shutdown():
        distance=math.sqrt((pose[0]-12.5)**2+(pose[1])**2)
        rospy.loginfo(pose)
        rospy.loginfo(distance)
        if(regions['front']<=d and regions['fright']<=d and regions['fleft']<=d):
            rospy.loginfo("first")
            velocity_msg.linear.x=0.3
            velocity_msg.angular.z=1.6
            pub.publish(velocity_msg)
            if(distance<=2):
                go_to_point(distance)
                if(distance==0 or pose[0]==12.5):
                    break
        elif(regions['front']>=d and regions['fright']>=d and regions['fleft']>=d):
            rospy.loginfo("sec")
            velocity_msg.linear.x=0.3
            pub.publish(velocity_msg)
            if(distance<=2):
                go_to_point(distance)
                if(distance==0 or pose[0]==12.5):
                    break
        elif(regions['front']<=d and regions['fright']<=d and regions['fleft']>=d):
            rospy.loginfo("third")
            velocity_msg.linear.x=0.3
            velocity_msg.angular.z=2
            pub.publish(velocity_msg)
            if(distance<=2):
                go_to_point(distance)
                if(distance==0 or pose[0]==12.5):
                    break
        elif(regions['front']<=d and regions['fright']<=d and regions['fleft']>=d and regions['left']>=d):
            rospy.loginfo("fourth")
            velocity_msg.linear.x=0.3
            velocity_msg.angular.z=1.6
            pub.publish(velocity_msg)
            if(distance<=2):
                go_to_point(distance)
                if(distance==0 or pose[0]==12.5):
                    break
        elif(regions['front']<=d and regions['fright']<=d and regions['right']>=d and regions['fleft']>=d and regions['left']>=d):
            rospy.loginfo("fifth")
            velocity_msg.linear.x=0.3
            velocity_msg.angular.z=1.6
            pub.publish(velocity_msg)
            if(distance<=2):
                go_to_point(distance)
                if(distance==0 or pose[0]==12.5):
                    break
        elif(regions['front']>=2 and regions['fright']<=2 and regions['right']<=2 and regions['fleft']>=2 and regions['left']>=2):
            rospy.loginfo("sixth")
            velocity_msg.linear.x=0.3
            velocity_msg.angular.z=-1.2
            pub.publish(velocity_msg)
            if(distance<=2):
                go_to_point(distance)
                if(distance==0 or pose[0]==12.5):
                    break
        elif(regions['fleft']<=d and regions['front']>=d and regions['fright']>=d and regions['right']>=d and regions['left']>=d):
            rospy.loginfo("last")
            velocity_msg.linear.x=0.3
            velocity_msg.angular.z=2
            pub.publish(velocity_msg)
            if(distance<=2):
                go_to_point(distance)
                if(distance==0 or pose[0]==12.5):
                    break
        elif(regions['fleft']<=d and regions['front']>=d and regions['fright']>=d and regions['right']>=d and regions['left']>=d):
            rospy.loginfo("last")
            velocity_msg.linear.x = 0      
            velocity_msg.angular.z=0.3
            pub.publish(velocity_msg)
            if(distance<=2):
                go_to_point(distance)
                if(distance==0 or pose[0]==12.5):
                    break
        elif(distance==0 ):
            break
        

    velocity_msg.linear.x=0
    velocity_msg.angular.z=0
    pub.publish(velocity_msg)
        
                
def odom_callback(data):
    global pose
    x = data.pose.pose.orientation.x;
    y = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    pose=[data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]
    yaw=pose[2]

def laser_callback(msg):
    global regions
    regions= {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:720]), 10)
    }

def go_to_point(distance):
    global pose
    rospy.init_node('ebot_controller')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan',LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rate = rospy.Rate(10) 
    velocity_msg = Twist()
    dy_ = y_goal-pose[1]
    dx_ = x_goal-pose[0]
    angle=math.atan(dy_/dx_)
    error=angle-pose[2]
    P_=2
    velocity_msg.linear.x = 0.3
    velocity_msg.angular.z = P_*error
    pub.publish(velocity_msg)
    print("distance for destination",distance)
    

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException():
        rospy.loginfo("Terminated")

    

