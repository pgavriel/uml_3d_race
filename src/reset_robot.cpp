#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelState.h"
#include <tf2/LinearMath/Quaternion.h>
#include <string>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

bool msg_received = false;
float x, y;
tf2::Quaternion orientation;

void spawn_callback(const geometry_msgs::PoseWithCovarianceStamped& spawn){
  x = spawn.pose.pose.position.x;
  y = spawn.pose.pose.position.y;
  orientation[2] = spawn.pose.pose.orientation.z;
  orientation[3] = spawn.pose.pose.orientation.w;
  msg_received = true;
}

int main(int argc, char **argv){
  // Setup ros node and NodeHandle
  ros::init(argc, argv, "robot_reset_node");
  ros::NodeHandle n;

  //Set defaults
  std::string model_name = "pioneer";
  std::string topic = "pioneer/spawn";
  n.getParam(ros::this_node::getName()+"/topic",topic);
  n.getParam(ros::this_node::getName()+"/model_name",model_name);

  ros::Subscriber sub = n.subscribe(topic, 1000, spawn_callback);
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>(model_name+"/cmd_vel", 1000);
  ros::Publisher odom_pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);

  // Node waits to receive a message from /spawn topics, then publishes the message
  // to /cmd_vel and /odom to reset the state of the robot to where it originally spawned.
  // Node then shuts itself down.
  ros::Rate loop_rate(1);
  while (ros::ok()){
    if(msg_received){
      // Twist and Pose defaults with zeroes, set what's relevant.
      geometry_msgs::Twist stopped;
      geometry_msgs::Pose pose;
      pose.position.x = x;
      pose.position.y = y;
      pose.orientation.z = orientation[2];
      pose.orientation.w = orientation[3];

      //Construct respawn ModelState
      gazebo_msgs::ModelState spawn;
      spawn.model_name = model_name;
      spawn.pose = pose;
      spawn.twist = stopped;
      spawn.reference_frame = "world";

      ROS_INFO("Stopping robot...");
      vel_pub.publish(stopped);
      ROS_INFO("Resetting robot position...");
      odom_pub.publish(spawn);

      ros::Duration(0.5).sleep();
      ros::shutdown();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
