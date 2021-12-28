#include "CrazyflieOnboardPIDController.hh"

#include <ignition/plugin/Register.hh>
extern "C" {
#include "controller_pid.h"
}

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
    controllerPidInit();
  ignmsg << "CrazyflieOnboardPIDController::PreUpdate" << std::endl;
  control_t control= {0};
  setpoint_t setpoint = {0};
  const sensorData_t sensors  = {0};
  const state_t state  = {0};
  const uint32_t tick  = 0;

  controllerPid(&control, &setpoint, &sensors, &state, tick);


}