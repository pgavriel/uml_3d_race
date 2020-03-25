#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

sensor_msgs::LaserScan scan;

void outputCallback(const sensor_msgs::LaserScan info){
  // Save scan information
  scan = info;
}

int main (int argc, char **argv)
{
ros::init(argc,argv,"mover_node");
ros::NodeHandle n;

ros::Subscriber sub = n.subscribe("/nerpio/frontscan_filtered",1000,outputCallback);
ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",100);

geometry_msgs::Twist command;
float speed, angle, left, right, diff, threshold;

ros::Rate loop_rate(20);
while (ros::ok()){
  ros::spinOnce();

  if (!scan.ranges.empty()){
    // scan.ranges is a vector filled with 240 distance measurements (in meters).
    // The blue rays coming off of the robot represent these values.
    // The total scan is 240 degrees centered in front of the robot, meaning
    // scan.ranges[119] is straight ahead, scan.ranges[0] is all the way to the
    // left, and scan.ranges[239] is all the way to the right.

    // Set Variables
    speed = 0.35f; // Speed to be published
    angle = 0.40f; // Angle to be published

    //ROS_INFO("DISTANCE AHEAD: %.2f",scan.ranges[119]);

    left = scan.ranges[74]; // Range measurement 45 Degrees left of center
    right = scan.ranges[164];// Range measurement 45 Degrees right of center
    diff = right-left;
    //ROS_INFO("LEFT: %.2f | RIGHT: %.2f | DIFF: %.2f",left, right, diff);

    // Setting a threshold here will reduce wiggling movement (try removing it)
    threshold = 0.25;
    if( abs(diff) < threshold)
      angle = 0.0;
    else if (diff < 0.0)
      angle = -angle;

    // Publish Commands
    // linear.x is forward/backward movement. Positive values for forward.
    command.linear.x  = speed;
    // angular.z is turning movement. Positive values turn left, negative turns right.
    command.angular.z = angle;
    //ROS_INFO(" PUBLISHING Speed:%.2f | Angle:%.2f\n",speed,angle);
    twist_pub.publish(command);
  }
  loop_rate.sleep();
}

return 0;
}
