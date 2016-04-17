#!/usr/bin/env python

import roslib; roslib.load_manifest('jarves_gazebo')
import rospy
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

def callback(msg, sac):
    # rospy.loginfo(rospy.get_caller_id() + " pose received!")

    #set goal
    goal = MoveBaseGoal()
    goal.target_pose.pose = msg.pose.pose
    goal.target_pose.header.frame_id = msg.header.frame_id
    goal.target_pose.header.stamp = msg.header.stamp

    #send goal
    sac.send_goal(goal)

def simple_follow():
    rospy.init_node('simple_follow') #, anonymous=True)
    leader_topic = rospy.get_param('~leader_topic', 'pose')
    movebase_ns = rospy.get_param('~movebase_ns', 'move_base')

    # rospy.loginfo(rospy.get_caller_id() + " pose_topic:" + pose_topic)
    # rospy.loginfo(rospy.get_caller_id() + " movebase_ns:" + movebase_ns)

    sac = SimpleActionClient(movebase_ns, MoveBaseAction)

    #get goal
    rospy.Subscriber(leader_topic, PoseWithCovarianceStamped, callback, sac)
    rospy.spin()

if __name__ == '__main__':
    simple_follow()
