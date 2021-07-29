# RoboticsND-Where-Am-I

Localization of a differential drive mobile robot in ROS using Adaptive Monte Carlo Localization inside a custom Gazebo world.

<br>

![Robot localization](images/localization.gif)



The robot uses odometry and laser scan data to localize itself through the `amcl` package, which is a probabilistic localization system for robots moving on a 2D plane. It implements the Adaptive Monte Carlo Localization approach, which uses a particle filter to track the pose of a robot against a known map and adjusts the number of particles over a time period for computational effieciency. 

The project consists of the following parts: 

* A ROS package that launches a custom robot model in a custom Gazebo world.
* The ROS `amcl` package and Navigation Stack to localize the robot.
* Exploration and tuning of specific parameters corresponding to each package to achieve the best possible localization results.

#### Map

The map of the environment was created with the [pgm_map_creator](https://github.com/udacity/pgm_map_creator) ROS package using ROS Kinetic.

#### Localization parameters

For a successful localization of the mobile robot, there are many parameters that need to be tuned for a specific robot and environment setup. Some of these parameters have been tuned and set in the config directory in order to make the robot accurately localize itself.





Rviz


Structure
---------

There are two packages in this project.
* **my_robot**: This package holds the robot and the Gazebo world.
* **where-am-i**:

Technologies
------------

The project was developed on Ubuntu 16.04 and 20.04 LTS with:
* ROS Kinetic/Noetic
* Gazebo 11.5.1

[Ubuntu 16.04 with ROS Kinetic was only used to create the map file of the environment. The rest of the project was developed on Ubuntu 20.04 with ROS Noetic.]


Dependencies
------------

The following dependencies need to be installed:
```sh
$ sudo apt-get update && sudo apt-get upgrade -y
$ sudo apt-get install ros-${ROS_DISTRO}-navigation
$ sudo apt-get install ros-${ROS_DISTRO}-map-server
$ sudo apt-get install ros-${ROS_DISTRO}-move-base
$ sudo apt-get install ros-${ROS_DISTRO}-amcl
```

Installation
------------

To run this project, you must have ROS and Gazebo installed.

#### Create a catkin_ws, if you do not already have one.
```sh
$ mkdir -p /catkin_ws/src/
$ cd catkin_ws/src/
$ catkin_init_workspace
```

#### Clone the project in catkin_ws/src/.
```sh
$ git clone https://github.com/elena-ecn/RoboticsND-Where-Am-I.git
```

#### Build the packages.
```sh
$ cd /catkin_ws/
$ catkin_make
```

#### Source your environment.
```sh
$ source devel/setup.bash
```

#### Launch the simulation environment.
```sh
$ roslaunch where_am_i world.launch
```

#### Launch the amcl localization.
In a new terminal (Ctrl+Shift+T), type:
```sh
$ source devel/setup.bash
$ roslaunch where_am_i amcl.launch
```

#### Send a Navigation Goal.
Send a 2D Nav Goal from RViz: Click the `2D Nav Goal` button in the toolbar, then click and drag on the map to send the goal to the robot. It will start moving and localize itself in the process.

<br>

![Robot navigation](images/navigation.gif)


License
-------

The contents of this repository are covered under the [MIT License](LICENSE).