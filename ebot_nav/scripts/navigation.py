#!/usr/bin/env python

# importing neccessary libraries
import rospy
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


class SeqMoveBase():
    def __init__(self):
        rospy.init_node('move_base_sequence')
        points_seq = [-9.1, -1.2, 0, 10.7, 10.5, 0,
                      12.6, -1.4, 0, 18.2, -1.4, 0, -2, 4, 0] #waypoints to be reached
        yaweulerangles_seq = [90, 0, 180, 180, 0] 
        quat_seq = list()
        self.pose_seq = list()  # goal poses list
        self.goal_cnt = 0      #keeps count of number of goal points reached
        
        for yawangle in yaweulerangles_seq:
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle * math.pi / 180, axes='sxyz'))))
        n = 3
        points = [points_seq[i:i + n] for i in range(0, len(points_seq), n)] #list of list of waypoints to be given to movebase
        rospy.loginfo(str(points))
        
        for point in points: #pose_seq contains goal pose
            self.pose_seq.append(Pose(Point(*point), quat_seq[n - 5]))
            n += 1

        self.client = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction)  # action client server
        wait = self.client.wait_for_server()
        
        if not wait:
            rospy.signal_shutdown("Action server not available!")
            return
        self.movebase_client()

    def active_cb(self):  #active call back function prints/logs goal pose
        rospy.loginfo("Goal pose " + str(self.goal_cnt + 1))

    def feedback_cb(self, feedback):  # this callback function prints current pose
        rospy.loginfo("Feedback for goal " +
                      str(self.goal_cnt) + ": " + str(feedback))

    def done_cb(self, status, result):
        self.goal_cnt += 1
        if status == 3:
            rospy.loginfo("Goal pose " + str(self.goal_cnt) + " reached")
            if self.goal_cnt < len(
                    self.pose_seq):  # if the bot has not reached the final goal
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose " +
                              str(self.goal_cnt + 1) + " to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.client.send_goal(
                    next_goal,
                    self.done_cb,
                    self.active_cb,
                    self.feedback_cb)
            else:
                rospy.signal_shutdown("Final goal pose reached!")
                return

        

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose " +
                      str(self.goal_cnt + 1) + " to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(
            goal,
            self.done_cb,
            self.active_cb,
            self.feedback_cb) #sending goal to move base 
        rospy.spin()


if __name__ == '__main__':  # main function
    try:
        SeqMoveBase()  # calling movebase class
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
