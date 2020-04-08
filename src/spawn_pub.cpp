#include "ros/ros.h"
#include <string>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char **argv){
  //Create ros node
  ros::init(argc, argv, "spawn_publisher");
  ros::NodeHandle n;

  //Set default values
  float x_position = 0.0;
  float y_position = 0.0;
  float z_rotation = 0.0;
  std::string topic = "spawn";
  int counter = 0;

  //Retrieve node parameters
  n.getParam(ros::this_node::getName()+"/x_position",x_position);
  n.getParam(ros::this_node::getName()+"/y_position",y_position);
  n.getParam(ros::this_node::getName()+"/z_rotation",z_rotation);
  n.getParam(ros::this_node::getName()+"/topic",topic);

  //Display node information on startup
  ROS_INFO("SPAWN_PUB | Point: (%.2f,%.2f)\tRotation: %.2f\tTopic: %s",x_position,y_position,z_rotation,topic.c_str());

  //Create the publisher object
  ros::Publisher spawn_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic, 1000);

  //Construct Spawn message
  geometry_msgs::PoseWithCovarianceStamped spawn;
  spawn.header.frame_id = "map";
  spawn.pose.pose.position.x = x_position;
  spawn.pose.pose.position.y = y_position;
  spawn.pose.pose.position.z = 0;
  tf2::Quaternion quaternion;
  quaternion.setRPY( 0, 0, z_rotation );
  spawn.pose.pose.orientation.z = quaternion[2];
  spawn.pose.pose.orientation.w = quaternion[3];

  //Publish Goal message once per second
  ros::Rate loop_rate(1);
  while (ros::ok()){
    spawn.header.seq = counter;
    spawn.header.stamp = ros::Time::now();
    spawn_pub.publish(spawn);
    counter++;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
