#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <stdlib.h> 
#include <iostream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool navigateTo(float x, float y, float z, float w) {
	
  
  MoveBaseClient ac("move_base", true);

  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = w;

  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return true;
  }
 
  return false;

}

int main(int argc, char** argv){

  ros::init(argc, argv, "pick_objects");

  std::cout << "Sending Pick Up Goal." << std::endl;
  bool pickUpZoneReached = navigateTo(-4.0, 8.0, 0.0, 1.0);
  
  if (!pickUpZoneReached) {
	  std::cout << "Failed to reach pickup zone." << std::endl;
	  return 0;
  } 
  
  std::cout << "Reach pickup zone, waiting 5 seconds." << std::endl;
  
  sleep(5);

  std::cout << "Sending Drop-off Goal" << std::endl;
  bool dropOffZoneReached = navigateTo(8.0, -5.0, 0.0, 1.0);
  if (dropOffZoneReached) {
	  std::cout << "Drop off zone reached." << std::endl;
  } else {
	  std::cout << "Drop-off Failed." << std::endl;
  }
  
  

  return 0;
}