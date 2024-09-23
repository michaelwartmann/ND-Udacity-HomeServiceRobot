#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client to send goal requests to the move_base server
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  // Tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 seconds for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Define the first goal (Pick-up Location)
  move_base_msgs::MoveBaseGoal pickup_goal;
  pickup_goal.target_pose.header.frame_id = "map";
  pickup_goal.target_pose.header.stamp = ros::Time::now();
  pickup_goal.target_pose.pose.position.x = 4.0; //  pick-up x coordinate
  pickup_goal.target_pose.pose.position.y = -5.0; // pick-up y coordinate
  pickup_goal.target_pose.pose.orientation.w = 1.0;

  // Send the pick-up goal
  ROS_INFO("Sending pick-up goal");
  ac.sendGoal(pickup_goal);

  // Wait for the result
  ac.waitForResult();

  // Check if the robot reached the pick-up zone
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("The robot reached the pick-up zone");
    // Pause for 5 seconds
    ros::Duration(5.0).sleep();
  }
  else{
    ROS_INFO("The robot failed to reach the pick-up zone");
  }

  // Define the second goal (Drop-off Location)
  move_base_msgs::MoveBaseGoal dropoff_goal;
  dropoff_goal.target_pose.header.frame_id = "map";
  dropoff_goal.target_pose.header.stamp = ros::Time::now();
  dropoff_goal.target_pose.pose.position.x = -6.0; // drop-off x coordinate
  dropoff_goal.target_pose.pose.position.y = -5.0; // drop-off y coordinate
  dropoff_goal.target_pose.pose.orientation.w = 1.0;

  // Send the drop-off goal
  ROS_INFO("Sending drop-off goal");
  ac.sendGoal(dropoff_goal);

  // Wait for the result
  ac.waitForResult();

  // Check if the robot reached the drop-off zone
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("The robot reached the drop-off zone");
  }
  else{
    ROS_INFO("The robot failed to reach the drop-off zone");
  }

  return 0;
}

