#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

//Goal is a custom message type defined in uml_3d_race/msg/Goal.msg
#include <uml_3d_race/Goal.h>

const float OFFICIAL_GOAL_TOL = 0.9;

float goal_x;
float goal_y;
float goal_tolerance;
int state = -1;
ros::Time start, finish;
float vel;
geometry_msgs::Point last_pos;

float distance(float x1, float y1, float x2, float y2){
  float dx = x2 - x1;
  float dy = y2 - y1;
  return sqrt(dx*dx + dy*dy);
}

void odom_callback(const nav_msgs::Odometry& msg){
  float goal_dist = distance(msg.pose.pose.position.x,msg.pose.pose.position.y,goal_x,goal_y);
  bool finished = (goal_dist < goal_tolerance);

  float odom_dist = distance(msg.pose.pose.position.x,msg.pose.pose.position.y,last_pos.x,last_pos.y);
  bool cheated = (odom_dist > 0.5);
  if(cheated){ state = 1; }
  //ROS_INFO("Odom Dist Traveled: %.2f",odom_dist);

  if(finished){
    if(abs(goal_tolerance-OFFICIAL_GOAL_TOL) > 0.01) { state = 2; ROS_INFO("TOL:%.5f",goal_tolerance);}
    finish = ros::Time::now();
    ROS_INFO("Goal Area Reached! [T=%.5f]", finish.toSec());
    float time = (finish - start).toSec();
    switch(state){
      case -1: ROS_INFO("How did you finish what you never started?");
               break;
      case 0: ROS_INFO("Finished in %f seconds!",time);
              break;
      case 1: ROS_INFO("Finished in %f seconds, but you did something weird.",time);
              break;
      case 2: ROS_INFO("Finished in %f seconds, but you changed the goal_tolerance.",time);
              break;
      default: ROS_INFO("Something broke.");
    }
    ros::shutdown();
  }
  last_pos = msg.pose.pose.position;
}

void vel_callback(const geometry_msgs::Twist& msg){
  vel = msg.linear.x;
  //ROS_INFO("VEL: %.3f\tSTATE: %d\t START: %.2f",vel,state,ros::Duration(start).toSec());
  if(state == -1 && vel > 0.01){
    state = 0;
    start = ros::Time::now();
    ROS_INFO("Starting timer now! [T=%.5f]", start.toSec());
  }
}

void goal_callback(const uml_3d_race::Goal& goal){
  goal_x = goal.x;
  goal_y = goal.y;
  goal_tolerance = goal.tolerance;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "referee");
  ros::NodeHandle n;

  ros::Subscriber odom_sub = n.subscribe("/pioneer/odom",1000,odom_callback);
  ros::Subscriber vel_sub = n.subscribe("/pioneer/cmd_vel",1000,vel_callback);
  ros::Subscriber goal_sub = n.subscribe("goal",1000,goal_callback);

  ros::Rate loop_rate(10);
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
