# Avatar Locomanipulation
This package contains code for performing kinematics and dynamics computations to enable locomanipulation with the NASA Valkyrie robot.

## System
Ubuntu 16.04 ROS Kinetic

## Dependency Installation
Add the following projects to your catkin workspace:
````
cd catkin_ws/src
git clone https://github.com/ipab-slmc/eigenpy_catkin
git clone --branch v1.0.1-with-pinocchio-v2.1.3 https://github.com/stevenjj/hpp-fcl_catkin.git
git clone --branch v2.1.3-with-hpp-fcl-v1.0.1 https://github.com/stevenjj/pinocchio_catkin
git clone https://github.com/stevenjj/val_model
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build
````

## Installation
To install this package
````
cd catkin_ws/src
git clone https://github.com/stevenjj/avatar_locomanipulation
catkin build
````

## Run Example Scripts
To run example locomanipulation trajectories. Launch one of the following:
````
roslaunch avatar_locomanipulation door_open.launch
roslaunch avatar_locomanipulation cart_push.launch
roslaunch avatar_locomanipulation door_open_different_cost.launch
roslaunch avatar_locomanipulation cart_push_different_cost.launch
roslaunch avatar_locomanipulation door_open_neural_net.launch  # Needs the neural network weights
roslaunch avatar_locomanipulation cart_push_neural_net.launch  # Needs the neural network weights
````
To visualize the reachability and locomanipulability regions. Launch one of the following:
````
roslaunch avatar_locomanipulation viz_reachability_contact_transition_space.launch
roslaunch avatar_locomanipulation viz_reachability_end_effector_space.launch
roslaunch avatar_locomanipulation viz_locomanipulability_contact_transition_space.launch
roslaunch avatar_locomanipulation viz_locomanipulability_ee_space.launch
````

## Get Neural Network Weights
Download the neural network weights from [here](https://drive.google.com/open?id=1N88vQLRJawx_CIurzRgQNeN3DB39xbGp) and extract the contents to the `nn_models` folder.

## Known Issue
Sometimes when running one of the example launch files, the planner immediately returns not finding a path. Relaunching the file again should work. I'm not sure of the issue as this bug appears to be intermittent on my system.

