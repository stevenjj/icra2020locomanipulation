# Avatar Locomanipulation
This package contains code for performing kinematics and dynamics computations to enable locomanipulation with the NASA Valkyrie robot.

## Dependency Installation
Add the following projects to your catkin workspace:
````
cd catkin_ws/src
git clone https://github.com/ipab-slmc/eigenpy_catkin
git clone --branch v1.0.1-with-pinocchio-v2.1.3 https://github.com/stevenjj/hpp-fcl_catkin.git
git clone --branch v2.1.3-with-hpp-fcl-v1.0.1 https://github.com/stevenjj/pinocchio_catkin
git clone https://github.com/stevenjj/val_model
````

## Installation
To install this package
````
cd catkin_ws/src
git clone https://github.com/stevenjj/avatar_locomanipulation
catkin build
````