#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <actionlib/client/simple_action_client.h>

double pickUpPos[2]  = {-4.0, 8.0};
double dropOffPos[2] = {8.0, -5.0};

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void send_goal_to_move_base(MoveBaseClient & ac, double x, double y, double orient)
{
  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "odom";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = orient;

  ROS_INFO("Sending goal ...");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Reached the pick up zone");
  else
    ROS_INFO("The base failed to reach the goal for some reason.");
}

int main(int argc, char** argv){
  ros::init(argc, argv, "pick_objects");
  MoveBaseClient ac("move_base", true);

  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  send_goal_to_move_base(ac, pickUpPos[0], pickUpPos[1], 1.0);

  sleep(5);

  send_goal_to_move_base(ac, dropOffPos[0], dropOffPos[1], 1.0);

  ROS_INFO("Reached the drop off zone");

  return 0;
}