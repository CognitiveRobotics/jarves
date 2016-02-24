//
// Author: Priyam Parashar
// Date: 02/22/16
//

//ros specific includes
#include "ros/ros.h"

//for sending goals
#include "actionlib/client/simple_action_client.h"

//messages
#include "move_base_msgs/MoveBaseAction.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

//global variables
geometry_msgs::PoseWithCovarianceStamped robot0_curr_pose;

//typedefs
typedef actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> MoveBaseClient;

//calbacks
void update_position(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
  robot0_curr_pose = *msg;
}

int main(int argc, char **argv) {

  //initialize node
  ros::init(argc, argv, "guide_follower");
  ROS_INFO("Following the guide robot");
  ros::NodeHandle nh;

  //local variables
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";

  //spin frequency
  ros::Rate r1(1), r2(50);

  //subscribe to robot_0's position
  ros::Subscriber amclSub = nh.subscribe("/robot_0/amcl_pose", 10, update_position);

  //connect to robot_1's move_base
  MoveBaseClient ac("/robot_1/move_base", true);

  //wait for the action server to come up
  ROS_INFO("Waiting for the move_base action server to come up");
  while (!ac.waitForServer(ros::Duration(5.0)) && ros::ok()) {
    ROS_INFO("...");
    ros::spinOnce();
    r2.sleep();
  }
  ROS_INFO("Client connected");

  while(ros::ok()){

    //send new location of robot_0 to robot_1 using movebaseclient
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = robot0_curr_pose.pose.pose.position.x;
    goal.target_pose.pose.position.y = robot0_curr_pose.pose.pose.position.y;
    goal.target_pose.pose.orientation.z = robot0_curr_pose.pose.pose.orientation.z;
    goal.target_pose.pose.orientation.w = robot0_curr_pose.pose.pose.orientation.w;
    goal.target_pose.pose.orientation.x = robot0_curr_pose.pose.pose.orientation.x;
    goal.target_pose.pose.orientation.y = robot0_curr_pose.pose.pose.orientation.y;

    ac.sendGoal(goal);
    
    ros::spinOnce();
    r1.sleep();
  }

  return 0;
}
