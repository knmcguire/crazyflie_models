#include <ignition/gazebo/System.hh>

namespace crazyflie_onboard_pid_controller
{
  class CrazyflieOnboardPIDController:
    // This class is a system.
    public ignition::gazebo::System,
    // This class also implements the ISystemPreUpdate, ISystemUpdate,
    // and ISystemcmak interfaces.
    public ignition::gazebo::ISystemPreUpdate
  {
    public: CrazyflieOnboardPIDController();
    public: ~CrazyflieOnboardPIDController() override;
    public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;
  };
}