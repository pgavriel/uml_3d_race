#include "ros/ros.h"
#include <string>

//Spawn is a custom message type defined in uml_3d_race/msg/Spawn.msg
#include <uml_3d_race/Spawn.h>

int main(int argc, char **argv){
  //Create ros node
  ros::init(argc, argv, "spawn_publisher");
  ros::NodeHandle n;

  //Set default values
  float x_position = 0.0;
  float y_position = 0.0;
  float z_rotation = 0.0;
  std::string topic = "spawn";

  //Retrieve node parameters
  n.getParam(ros::this_node::getName()+"/x_position",x_position);
  n.getParam(ros::this_node::getName()+"/y_position",y_position);
  n.getParam(ros::this_node::getName()+"/z_rotation",z_rotation);
  n.getParam(ros::this_node::getName()+"/topic",topic);

  //Display node information on startup
  ROS_INFO("SPAWN_PUB | Point: (%.2f,%.2f)\tRotation: %.2f\tTopic: %s",x_position,y_position,z_rotation,topic.c_str());

  //Create the publisher object
  ros::Publisher spawn_pub = n.advertise<uml_3d_race::Spawn>(topic, 1000);

  //Construct Spawn message
  uml_3d_race::Spawn spawn;
  spawn.x_position = x_position;
  spawn.y_position = y_position;
  spawn.z_rotation = z_rotation;

  //Publish Goal message once per second
  ros::Rate loop_rate(1);
  while (ros::ok()){
    spawn_pub.publish(spawn);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
