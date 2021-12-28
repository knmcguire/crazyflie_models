#include "CrazyflieOnboardPIDController.hh"

#include <ignition/plugin/Register.hh>


IGNITION_ADD_PLUGIN(
    crazyflie_onboard_pid_controller::CrazyflieOnboardPIDController,
    ignition::gazebo::System,
    crazyflie_onboard_pid_controller::CrazyflieOnboardPIDController::ISystemPreUpdate)

using namespace crazyflie_onboard_pid_controller;

CrazyflieOnboardPIDController::CrazyflieOnboardPIDController()
{
}

CrazyflieOnboardPIDController::~CrazyflieOnboardPIDController()
{
}

void CrazyflieOnboardPIDController::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  ignmsg << "CrazyflieOnboardPIDController::PreUpdate" << std::endl;
}