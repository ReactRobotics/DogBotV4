# dogbot_gazebo

Controls operation of simulated DogBot(s) in gazebo.  The robot description is in the dogbot_description package.

# Quick start

<pre>
cd ROS
catkin build
source devel/setup.bash
roslaunch dogbot_gazebo gztest.launch
</pre>


# Launch files

Top-level files start with gz

Other files are components, intended to be included into the higher-level files (esp. to set the namespace correctly)

# Prerequisites

Ensure you have gazebo-ros control packages installed, http://wiki.ros.org/gazebo_ros_control

<pre>sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control</pre>
