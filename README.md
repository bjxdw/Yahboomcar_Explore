# Yahboomcar_Explore based on RRT Exploration for Ubuntu 18.04 ROS Melodic

## Requirements

The following libraries are required to be installed before proceeding to run the code

    $ sudo apt-get install ros-melodic-gmapping
    $ sudo apt-get install ros-melodic-navigation
    $ sudo apt-get install python-opencv
    $ sudo apt-get install python-numpy
    $ sudo apt-get install python-scikits-learn
    $ sudo apt-get install ros-melodic-teb-local-planner

## Installation Process

Create a new folder called "catkin_explore/src" by executing the following comment:

    $ sudo mkdir -p ~/catkin_explore/src
    $ cd ~/catkin_explore/src/
    $ git clone https://github.com/hikashi/multi-robot-rrt-exploration-melodic.git
    $ cd ~/catkin_explore
    $ catkin_make

## Execution for Single Robot (Simulation)

The program can be executed using the following comments in three different terminals:

Terminal 1

     # roscore 

Terminal 2

     # source ~/catkin_explore/devel/setup.bash 
     # roslaunch ros_multitb3 single_yahboom.launch

Terminal 3

     # source ~/catkin_explore/devel/setup.bash 
     # roslaunch rrt_exploration single_yahboom_exploration.launch 

## Execution for Multirobot (Simulation)

The program can be executed using the following comments in three different terminals:

Terminal 1

     # roscore 

Terminal 2

     # source ~/catkin_explore/devel/setup.bash 
     # roslaunch ros_multitb3 multi_yahboom2.launch 

Terminal 3

     # source ~/catkin_explore/devel/setup.bash  
     # roslaunch rrt_exploration multi_yahboom_exploration.launch 

## Execution for Single Robot (Real robot)

The program can be executed using the following comments in three different terminals:

On yahboomcar_X3:
     # roslaunch yahboomcar_nav laser_bringup.launch

On Computer:
Terminal 1

     # roscore 

Terminal 2

     # source ~/catkin_explore/devel/setup.bash 
     # roslaunch ros_multitb3 single_real_robot.launch

Terminal 3

     # source ~/catkin_explore/devel/setup.bash 
     # roslaunch rrt_exploration single_yahboom_exploration.launch 

## Execution for Multirobot (real_robot)

The program can be executed using the following comments in three different terminals:

On yahboomcar_X3:
     # roslaunch yahboomcar_nav laser_bringup.launch

On Computer:
Terminal 1

     # roscore 

Terminal 2

     # source ~/catkin_explore/devel/setup.bash 
     # roslaunch ros_multitb3 multi_real_robot.launch 

Terminal 3

     # source ~/catkin_explore/devel/setup.bash  
     # roslaunch rrt_exploration multi_yahboom_exploration.launch 
