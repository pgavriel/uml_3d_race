#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelState.h"
#include <tf2/LinearMath/Quaternion.h>
#include <string>

//Spawn is a custom message type defined in uml_3d_race/msg/Spawn.msg
#include <uml_3d_race/Spawn.h>

bool msg_received = false;
float x, y, yaw;

void spawn_callback(const uml_3d_race::Spawn& spawn){
  x = spawn.x_position;
  y = spawn.y_position;
  yaw = spawn.z_rotation;
  msg_received = true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "robot_reset_node");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("spawn", 1000, spawn_callback);
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
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
      tf2::Quaternion quaternion;
      quaternion.setRPY( 0, 0, yaw );
      pose.orientation.z = quaternion[2];
      pose.orientation.w = quaternion[3];

      //Construct respawn ModelState
      gazebo_msgs::ModelState spawn;
      spawn.model_name = "Pioneer";
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
