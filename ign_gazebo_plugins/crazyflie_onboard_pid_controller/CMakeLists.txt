cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(ignition-cmake2 REQUIRED)

project(CrazyflieOnboardPIDController)

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

find_package(ignition-gazebo6 REQUIRED)
add_library(CrazyflieOnboardPIDController SHARED CrazyflieOnboardPIDController.cc controller_pid.c position_controller_pid.c attitude_pid_controller.c pid.c)
set_property(TARGET CrazyflieOnboardPIDController PROPERTY CXX_STANDARD 17)
target_link_libraries(CrazyflieOnboardPIDController
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo6::ignition-gazebo6)