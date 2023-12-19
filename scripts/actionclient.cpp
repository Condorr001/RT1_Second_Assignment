#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_2_2023/PlanningAction.h>
#include <iostream>

int main (int argc, char **argv)
{
  //declaring the coordinates to get in input and send to the action server
  float x,y;
  ros::init(argc, argv, "action_client");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<assignment_2_2023::PlanningAction> ac("action_client", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  //notify that the server started and the client detected it
  ROS_INFO("Action server started.\n");
  
  //asking the user for the goal target to reach
  std::cout << "Enter the x and y target point coordinates as floats: ";
  std::cin >> x;
  std::cin >> y;
  
  // send the goal to the server as it is defined in the .action file
  /*In the .action file, the goal is defined as a "geometry_msg/PoseStamped element.
    This message is composed by the following fields:
	std_msgs/Header header
	  uint32 seq
	  time stamp
	  string frame_id
	geometry_msgs/Pose pose
	  geometry_msgs/Point position
	    float64 x
	    float64 y
	    float64 z
	  geometry_msgs/Quaternion orientation
	    float64 x
	    float64 y
	    float64 z
	    float64 w
	    
    What we need to send to the server are the x and y position coordinates of the target point
 */
  assignment_2_2023::PlanningGoal goal;
  goal.target_pose.pose.position.x = 20.0;
  goal.target_pose.pose.position.y = 30.0;
  ac.sendGoal(goal);

  //wait for the action to returnn up to a timeout limit set to 60 seconds
  bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));

  if (finished_before_timeout)
  {
    //if the ac finished before the timeout, return the finished state
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
