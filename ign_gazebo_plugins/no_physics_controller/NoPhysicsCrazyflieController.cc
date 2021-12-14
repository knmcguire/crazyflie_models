 /*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include "NoPhysicsCrazyflieController.hh"

#include <ignition/gazebo/components/Pose.hh>
#include <ignition/plugin/Register.hh>


#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include "ignition/gazebo/components/PhysicsEnginePlugin.hh"
#include <ignition/gazebo/Model.hh>
#include <ignition/plugin/Loader.hh>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/Util.hh"
#include <ignition/msgs/twist.pb.h>

#include <ignition/transport/Node.hh>
#include <math.h>       /* cos */


IGNITION_ADD_PLUGIN(
    no_physics_crazyflie_controller::NoPhysicsCrazyflieController,
    ignition::gazebo::System,
    no_physics_crazyflie_controller::NoPhysicsCrazyflieController::ISystemConfigure,
    no_physics_crazyflie_controller::NoPhysicsCrazyflieController::ISystemPreUpdate)
using namespace no_physics_crazyflie_controller;

//////////////////////////////////////////////////
NoPhysicsCrazyflieController::NoPhysicsCrazyflieController()
{
}

//////////////////////////////////////////////////
NoPhysicsCrazyflieController::~NoPhysicsCrazyflieController()
{
}

//////////////////////////////////////////////////
void NoPhysicsCrazyflieController::Configure(const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &/*_eventMgr*/)
{
  this->entity = _entity;
  ignmsg << "Controller for entity [" << _entity << "]" << std::endl;
  this->previous_position = {0, 0, 0};

    std::string topic{"/cmdvel"};
  this->node.Subscribe(topic, &NoPhysicsCrazyflieController::OnTwist, this);


}

//////////////////////////////////////////////////
void NoPhysicsCrazyflieController::OnTwist(
    const ignition::msgs::Twist &_msg)
{
  std::lock_guard<std::mutex> lock(this->cmdVelMsgMutex);
  this->cmdVelMsg = _msg;
  //ignmsg <<this->cmdVelMsg<<std::endl;

}

//////////////////////////////////////////////////
void NoPhysicsCrazyflieController::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  ignition::math::Vector3d linearVelCmd({0,0,0});
  ignition::math::Vector3d angularVelCmd({0,0,0});

    std::lock_guard<std::mutex> lock(this->cmdVelMsgMutex);
    if (this->cmdVelMsg.has_value())
    {
      linearVelCmd = ignition::msgs::Convert(this->cmdVelMsg->linear());
      angularVelCmd = ignition::msgs::Convert(this->cmdVelMsg->angular());

    }



  auto PoseComp =
      _ecm.Component<ignition::gazebo::components::Pose>(this->entity);

  ignition::gazebo::Model model(this->entity);


  auto sec = double(std::chrono::duration_cast<std::chrono::nanoseconds>(
    _info.simTime).count())*10e-9;
  auto dtime = sec - this->previous_time;


  ignition::math::Pose3 pose = PoseComp->Data();
  ignition::math::Vector3d v = ignition::math::Vector3(pose.Pos());
    
  double x,y,z;
  double roll, pitch, yaw;

  x = v.X(); // x coordinate
  y = v.Y(); // y coordinate
  z = v.Z(); // z coordinate
  yaw = pose.Yaw();



  double cosyaw = cos(yaw);
  double sinyaw = sin(yaw);

  double state_body_vx = x* cosyaw + y * sinyaw;
  double state_body_vy = -x * sinyaw + y * cosyaw;
  double pos_x_cmd = state_body_vx + linearVelCmd[0]*dtime;
  double pos_y_cmd = state_body_vy + linearVelCmd[1]*dtime;
  double pos_z_cmd = z + linearVelCmd[2]*dtime;

  double pos_x_global_cmd = pos_x_cmd*cosyaw - pos_y_cmd*sinyaw;
  double pos_y_global_cmd = pos_x_cmd*sinyaw + pos_y_cmd*cosyaw;

  double yaw_cmd = yaw + angularVelCmd[2]*dtime;


  ignition::math::Pose3d cmdPose;
  cmdPose.Set(pos_x_global_cmd, pos_y_global_cmd, pos_z_cmd, 0, 0, yaw_cmd);

  model.SetWorldPoseCmd(_ecm, cmdPose);

  this->previous_position = v;
  this->previous_time = sec;
    
    
}
