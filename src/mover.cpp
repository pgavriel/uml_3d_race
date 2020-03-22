#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

ros::Publisher twist_pub;

void outputCallback(const sensor_msgs::LaserScan info)
{
//Variables
int s = info.ranges.size();
ROS_INFO("Size: %d", s);
float speed    = 0.35f;
float left  	 = (info.ranges[30]+info.ranges[60]+info.ranges[90])/3;
ROS_INFO("LEFT: %.2f",left);
float right 	 = (info.ranges[210]+info.ranges[180]+info.ranges[150])/3;
ROS_INFO("RIGHT: %.2f", right);
float angle 	 = (right - left)/2;

//Publishing
geometry_msgs::Twist command;
command.linear.x  = speed;
command.angular.z = angle;
ROS_INFO("\nPUBLISHING Speed:%.2f | Angle:%.2f\n",speed,angle);
twist_pub.publish(command);
}

int main (int argc, char **argv)
{
ros::init(argc,argv,"mover_node");
ros::NodeHandle p;
twist_pub = p.advertise<geometry_msgs::Twist>("/cmd_vel",100);

ros::Subscriber sub = p.subscribe("/nerpio/frontscan_filtered",1000,outputCallback);
ros::spin();

return 0;
}
