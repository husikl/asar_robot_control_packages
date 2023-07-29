#ifndef _FORCEPS_HW_H
#define _FORCEPS_HW_H

// #include <boost/scoped_ptr.hpp>

// Ros related
#include <ros/ros.h>

// Hardware interface related
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
// #include <joint_limits_interface/joint_limits_urdf.h>

// Eigen
#include <Eigen/Dense>

using namespace Eigen;

namespace forceps_nu
{

  class ForcepsHW : public hardware_interface::RobotHW
  {

public:
protected:
    // Ros-control interfaces
    hardware_interface::JointStateInterface    joint_state_interface_;
    hardware_interface::PositionJointInterface position_interface_;
    hardware_interface::VelocityJointInterface velocity_interface_;
    hardware_interface::EffortJointInterface   effort_interface_;

    // joint_limits_interface::PositionJointSoftLimitsInterface
    //     position_limits_interface_;
    // joint_limits_interface::PositionJointSaturationInterface
    //     position_saturation_interface_;
    // joint_limits_interface::EffortJointSoftLimitsInterface
    //     effort_limits_interface_;
    joint_limits_interface::EffortJointSaturationInterface
        effort_limits_interface_;

    // Configuration
    //   VectorXd encoder_ranges, effort_limits, velocity_limits;

    // Shared memory
    std::vector<std::string> joint_names_;
    int                      n_dof_;
    VectorXd                 joint_position_;
    VectorXd                 joint_velocity_;
    VectorXd                 joint_effort_;
    VectorXd                 joint_position_cmd_;
    VectorXd                 joint_velocity_cmd_;
    VectorXd                 joint_effort_cmd_;

    void jointInterfaceSetZero()
    {
      joint_position_.setZero();
      joint_velocity_.setZero();
      joint_effort_.setZero();
      joint_position_cmd_.setZero();
      joint_velocity_cmd_.setZero();
      joint_effort_cmd_.setZero();
    }

    void jointInterfaceResize(int n_dof)
    {
      joint_position_.resize(n_dof);
      joint_velocity_.resize(n_dof);
      joint_effort_.resize(n_dof);
      joint_position_cmd_.resize(n_dof);
      joint_velocity_cmd_.resize(n_dof);
      joint_effort_cmd_.resize(n_dof);
    }

  }; //

} // namespace forceps_nu

#endif