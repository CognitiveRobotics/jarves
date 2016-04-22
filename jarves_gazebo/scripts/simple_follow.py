#!/usr/bin/env python

import roslib; roslib.load_manifest('jarves_gazebo')
import rospy
import numpy
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

global_goal = MoveBaseGoal()

def goal_callback(msg, sac):
    # rospy.loginfo(rospy.get_caller_id() + " pose received!")
    global global_goal

    #set goal
    goal = MoveBaseGoal()
    goal.target_pose.pose = msg.pose.pose
    goal.target_pose.header.frame_id = msg.header.frame_id
    goal.target_pose.header.stamp = msg.header.stamp

    #store value
    global_goal = goal

    #send goal
    sac.send_goal(goal)

def amcl_callback(msg, sac):
    #distance between current pose and goal
    global global_goal

    x1 = msg.pose.pose.position.x
    y1 = msg.pose.pose.position.y
    x2 = global_goal.target_pose.pose.position.x
    y2 = global_goal.target_pose.pose.position.y
    a = numpy.array([x1, y1])
    b = numpy.array([x2, y2])
    distance = numpy.linalg.norm(a-b)
    rospy.logdebug("self location= (%f, %f)", x1,y1)
    rospy.logdebug("leader location = (%f, %f)", x2, y2)
    rospy.logdebug("distance = %f",distance)

    if distance > 0.5 and distance < 1.5:
        rospy.loginfo("cancelling goal pursue. distance = %f",distance)
        sac.cancel_goal()

def simple_follow():
    rospy.init_node('simple_follow') #, anonymous=True)
    leader_topic = rospy.get_param('~leader_topic', 'pose')
    movebase_ns = rospy.get_param('~movebase_ns', 'move_base')
    amcl_pose = rospy.get_param('~follower_topic', 'pose')

    # rospy.loginfo(rospy.get_caller_id() + " pose_topic:" + pose_topic)
    # rospy.loginfo(rospy.get_caller_id() + " movebase_ns:" + movebase_ns)

    sac = SimpleActionClient(movebase_ns, MoveBaseAction)

    #get goal
    rospy.Subscriber(leader_topic, PoseWithCovarianceStamped, goal_callback, sac)

    #get own position
    rospy.Subscriber(amcl_pose, PoseWithCovarianceStamped, amcl_callback, sac)

    rospy.spin()

if __name__ == '__main__':
    simple_follow()
