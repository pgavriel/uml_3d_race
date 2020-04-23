#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

sensor_msgs::LaserScan scan;

void scanCallback(const sensor_msgs::LaserScan info){
  // Save scan information
  scan = info;
  for(int i = 0; i < scan.ranges.size() ; i++){
    if(info.ranges[i]>10.0)
      scan.ranges[i] = 10;
  }
}

int main (int argc, char **argv){
// Setup ros node and NodeHandle
ros::init(argc,argv,"mover_node");
ros::NodeHandle n;

// Set up topic variables
std::string model_name = "obs_bot";
n.getParam(ros::this_node::getName()+"/model_name",model_name);
std::string scantopic = model_name+"/scan";
std::string cmdtopic = model_name+"/cmd_vel";

// Set up movement variables
float max_speed = 1.0;
n.getParam(ros::this_node::getName()+"/max_speed",max_speed);
float max_angle = 1.0;
n.getParam(ros::this_node::getName()+"/max_angle",max_angle);
max_angle = abs(max_angle);

ROS_INFO("Bot Name: %s\t MaxSpeed: %.2f\t MaxAngle: %.2f",model_name.c_str(),max_speed,max_angle);

// Setup subscriber to poll the laser scan topic being published by our robot
ros::Subscriber sub = n.subscribe(scantopic,1000,scanCallback);
// Setup publisher object to publish movement commands to our robot
ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>(cmdtopic,100);

// Declare working variables
geometry_msgs::Twist command;
float speed, angle, left, right, diff, threshold;
int cointoss;
bool choosing = false;

// Set loop rate
ros::Rate loop_rate(20);
// Main control loop
while (ros::ok()){
  // Poll subscribed laser topic
  ros::spinOnce();

  if (!scan.ranges.empty()){

    // Set Variables
    speed = 0.35f; // Speed to be published
    angle = 0.40f; // Angle to be published

    //ROS_INFO("DISTANCE AHEAD: %.2f",scan.ranges[119]);

    left = (scan.ranges[99]+scan.ranges[79]+scan.ranges[59]+scan.ranges[29]+scan.ranges[0])/5;
    right = (scan.ranges[119]+scan.ranges[139]+scan.ranges[159]+scan.ranges[189]+scan.ranges[219])/5;
    diff = right-left;
    ROS_INFO("LEFT: %.2f | RIGHT: %.2f | DIFF: %.2f",left, right, diff);

    //Speed control
    speed = (left+right)/2;
    speed *= 0.25;
    if (speed > max_speed) speed = max_speed;

    // Turning control
    threshold = 0.1;
    if( abs(diff) < threshold){
      if(speed < (max_speed*0.1)){
        if(!choosing){
          cointoss = std::rand() % 3 - 1;
          if (cointoss != 0) choosing = true;
        }
        ROS_INFO("COIN: %d",cointoss);
        angle = cointoss;
      }
      else
        angle = 0.0;
    }
    else{
      choosing = false;
      angle = diff;
    }

    if(angle < -max_angle) angle = -max_angle;
    if(angle > max_angle)  angle = max_angle;

    // Publish Commands
    // linear.x is forward/backward movement. Positive values for forward.
    command.linear.x  = speed;
    // angular.z is turning movement. Positive values turn left, negative turns right.
    command.angular.z = angle;
    ROS_INFO(" PUBLISHING Speed:%.2f | Angle:%.2f\n",speed,angle);
    twist_pub.publish(command);
  }
  loop_rate.sleep();
}

return 0;
}
