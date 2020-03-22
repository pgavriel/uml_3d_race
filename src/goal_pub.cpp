#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <string>

int main(int argc, char **argv){
  ros::init(argc, argv, "goal_publisher");
  ros::NodeHandle n;

  float goal_x = 0.0;
  float goal_y = 0.0;
  float goal_z = 0.9;
  std::string topic = "goal";

  n.getParam(ros::this_node::getName()+"/goal_x",goal_x);
  n.getParam(ros::this_node::getName()+"/goal_y",goal_y);
  n.getParam(ros::this_node::getName()+"/goal_z",goal_z); //Used for goal tolerance and spawn rotation.
  n.getParam(ros::this_node::getName()+"/topic",topic);
  ROS_INFO("PUBLISHING | Point: (%.2f,%.2f)\tTolerance: %.2f\tTopic: %s",goal_x,goal_y,goal_z,topic.c_str());

  ros::Publisher goal_pub = n.advertise<geometry_msgs::Point>(topic, 1000);

  geometry_msgs::Point goal;
  goal.x = goal_x;
  goal.y = goal_y;
  goal.z = goal_z;

  ros::Rate loop_rate(1);
  while (ros::ok()){
    goal_pub.publish(goal);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
