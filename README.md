# ROS 3D RACE
## Setup:
1. Clone this repository into your catkin workspace.
  > cd ~/[CATKIN_WS]/src   
  > git clone https://github.com/pgavriel/uml_3d_race.git    
2. Build your catkin workspace.   
  > cd ..  
  > catkin build   
3. Source the workspace.  
  > source devel/setup.bash   
4. Run the setup script for uml_3d_race. (While still in your workspace directory)
  > rosrun uml_3d_race setup.sh  

At this point, if all goes well, the package should be ready to run.  
**NOTE BEFORE STARTING:** Sometimes Gazebo can be a bit finicky, and may fail to launch properly for an array of reasons. If something goes wrong, Ctrl+c and try again a few times. If the problem persists there may be an actual issue that needs to be resolved first, but unless you've already started making changes within this package, that shouldn't be the case.  

To Start:  
1. In a terminal, launch Gazebo (Launch file includes, gazebo, gazebo world to load, robot model, and more):  
  > roslaunch uml_3d_race level1.launch  
2. In a second terminal, launch the referee and the mover_node which controls the robots movement:  
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

## Useful Resources:  
[ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)  
[Gazebo Tutorials](http://gazebosim.org/tutorials)  
[ROS .launch file documentation](http://wiki.ros.org/roslaunch/XML)  
[ROS Navigation Tutorials](http://wiki.ros.org/navigation/Tutorials)  
[SDF (Simulation Description Format) Specification for Gazebo Models](http://sdformat.org/spec)  
[OGRE Material Script Documentation (Gazebo Textures)](https://ogrecave.github.io/ogre/api/1.12/_material-_scripts.html)  
[Your best friend](http://google.com)  
