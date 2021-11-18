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


## Current status
It's pretty much only the visualization and collision model, so it's not yet really flying. 
