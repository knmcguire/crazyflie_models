# crazyflie_models

This is a repository to host Crazyflie based models.

Supported Simulators:
* ignition gazebo (Tested on Fortress)

Supported OS
* Ubuntu 20.04 (tested)

Inspired by the quadcopter example of https://cyaninfinite.com/getting-started-with-ignition-gazebo/

## Current status

Still in development! 
## Getting started:

1- First install ignition gazebo: https://ignitionrobotics.org/docs/fortress/install

2- Clone this repo: 
`git clone git@github.com:knmcguire/crazyflie_models.git`

3- Put this repo in your  ~/.bashrc and source it in your terminal

`export IGN_GAZEBO_RESOURCE_PATH="path/to/your/repo/crazyflie_models/"`

4- Try out the crazyflie world with:
`ign gazebo crazyflieworld.sdf`

5- Try out the control (still needs to be tuned)
`ign topic -t "/crazyflie/gazebo/command/twist" -m ignition.msgs.Twist -p "linear: {x:0 y: 0 z: 0.1} angular {z: 0}"`

### Only controlling motors without a controller

First go to crazyflie/model.sdf, comment out the multicopter system plugin
```
      <!--plugin
        filename="ignition-gazebo-multicopter-control-system"
        name="ignition::gazebo::systems::MulticopterVelocityControl">
        ...
      </plugin-->
```
Then try an take off with sending a command to the motors:
`ign topic -t /crazyflie/gazebo/command/motor_speed --msgtype ignition.msgs.Actuators -p 'velocity:[2500,2500,2500,2500]'`

      
## No physics controller (ignition gazebo fortress)

Simple plugin to make crazyflie take off without physics.

To build the plugin:
1- go to:
`cd ign_gazebo_plugins/no_physics_controller`

2- build the plugin
    mkdir build
    cd build
    cmake ..
    make

3- Source the plugin path inside the build folder
```
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=`pwd`
```

4- With the gazebo model path as configured in the previous section, do this:
`ign gazebo -v 4 crazyflieworld_nophysics.sdf`

5- Control the z axis velocity of the crazyflies with:
`ign topic -t "/cmdvel" -m ignition.msgs.Twist -p "linear: {x:0 y: 0 z: 0.1} angular {z: 0}"`

6- you can now control the linear body fixed velocity and yaw rate

## Actor animation (gazebo 11)

Another way to control the crazyflie if one is only interested in position is through the actor frame work. This does NOT work on gazebo ignition as it's actor animation only work with skeleton intergrated for the time being, which does not make sense for the crazyflie. So install gazebo (version 11), make sure that "GAZEBO_MODEL_PATH" is correctly configured to also include crazyflie models, and try:

`gazebo crazyflieworld_actor.world`

