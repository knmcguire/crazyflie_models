# crazyflie_models

This is a repository to host Crazyflie based models.

Supported Simulators:
* ignition gazebo (Tested on Fortress)

Supported OS
* Ubuntu 20.04 (tested)

Inspired by the quadcopter example of https://cyaninfinite.com/getting-started-with-ignition-gazebo/


## Getting started:

1- First install ignition gazebo: https://ignitionrobotics.org/docs/fortress/install

2- Clone this repo: 
`git clone git@github.com:knmcguire/crazyflie_models.git`

3- Put this repo in your  ~/.bashrc and source it in your terminal

`export IGN_GAZEBO_RESOURCE_PATH="path/to/your/repo/crazyflie_models/"`

4- Try out the crazyflie world with:
`ign gazebo crazyflieworld.sdf`

5- Turn the rotors and take off with:
`ign topic -t /crazyflie/gazebo/command/motor_speed --msgtype ignition.msgs.Actuators -p 'velocity:[2500,2500,2500,2500]'`

6- Try out the control (still needs to be tuned)
`ign topic -t "/crazyflie/gazebo/command/twist" -m ignition.msgs.Twist -p "linear: {x:0 y: 0 z: 0.1} angular {z: 0}"`

## Actor animiation (gazebo 11)

Another way to control the crazyflie if one is only interested in position is through the actor frame work. This does NOT work on gazebo ignition as it's actor animation only work with skeleton intergrated for the time being, which does not make sense for the crazyflie. So install gazebo (version 11), make sure that "GAZEBO_MODEL_PATH" is correctly configured to also include crazyflie models, and try:

`gazebo crazyflieworld_actor.world`

## Current status
It's pretty much only the visualization and collision model, so it's not yet really flying. 
