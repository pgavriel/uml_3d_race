#include "ros/ros.h"
#include <string>

//Goal is a custom message type defined in uml_3d_race/msg/Goal.msg
#include <uml_3d_race/Goal.h>

int main(int argc, char **argv){
  //Create ros node
  ros::init(argc, argv, "goal_publisher");
  ros::NodeHandle n;

  //Set default values
  float goal_x = 0.0;
  float goal_y = 0.0;
  float goal_tolerance = 0.9;
  std::string topic = "goal";

  //Retrieve node parameters
  n.getParam(ros::this_node::getName()+"/goal_x",goal_x);
  n.getParam(ros::this_node::getName()+"/goal_y",goal_y);
  n.getParam(ros::this_node::getName()+"/goal_tolerance",goal_tolerance);
  n.getParam(ros::this_node::getName()+"/topic",topic);

  //Display node information on startup
  ROS_INFO("GOAL_PUB | Point: (%.2f,%.2f)\tTolerance: %.2f\tTopic: %s",goal_x,goal_y,goal_tolerance,topic.c_str());

  //Create the publisher object
  ros::Publisher goal_pub = n.advertise<uml_3d_race::Goal>(topic, 1000);

  //Construct Goal message
  uml_3d_race::Goal goal;
  goal.x = goal_x;
  goal.y = goal_y;
  goal.tolerance = goal_tolerance;

  //Publish Goal message once per second
  ros::Rate loop_rate(1);
  while (ros::ok()){
    goal_pub.publish(goal);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
