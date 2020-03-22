# ROS 3D RACE
## Setup:
1. Clone this repository into your catkin workspace.
  > cd ~/[CATKIN_WS]/src   
  > git clone https://github.com/pgavriel/uml_3d_race.git    
1. Build your catkin workspace.   
  > cd ..  
  > catkin build   
1. Source the workspace.  
  > source devel/setup.bash   
1. Run the setup script for uml_3d_race.  
  > rosrun uml_3d_race setup.sh  

At this point, if all goes well, the package should be ready to run.  
To Start:  
1. In one terminal start roscore:  
  > roscore  
1. In a second terminal, launch Gazebo (Launch file includes, gazebo, gazebo world to load, robot model, and more):  
  > roslaunch uml_3d_race level1.launch  
1. In a third, launch the referee and the mover_node which controls the robots movement:  
  > roslaunch uml_3d_race race.launch  

Race.launch will start a new referee node (which records your time) and a mover_node (which reads data and sends movement commands to the robot).   
When things inevitably go wrong, shutdown the terminal running race.launch with 'Ctrl+c', and reset the robots position via:  
  > rosrun uml_3d_race reset_robot  
  
Then you may want to tweak your code, **rebuild the workspace (*catkin build*)**, and run race.launch to try again.  
When you feel like you have beaten the current level, shutdown the terminal running gazebo with 'Ctrl+c', and try the next level via:  
  >roslaunch uml_3d_race levelX.launch  

## Challenge:  
Your goal is to use the sensor data being received by the robot to navigate through increasingly complex stages in the fastest time possible.
Start with level1.launch and work your way up to levelX.launch. Write an algorithm that can find its way through any course!  

## Things to Explore:  
- How to create and use your own gazebo world course  
- How to use information from all the sensors  
- How to use custom textures  
