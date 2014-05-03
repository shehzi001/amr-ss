#include <ros/console.h>
#include "omni_velocity_controller.h"

OmniVelocityController::OmniVelocityController(double l_max_vel, double l_max_acc, double l_tolerance,
                                               double a_max_vel, double a_max_acc, double a_tolerance)
: l_max_vel_(l_max_vel)
, l_max_acc_(l_max_acc)
, l_tolerance_(l_tolerance)
, a_max_vel_(a_max_vel)
, a_max_acc_(a_max_acc)
, a_tolerance_(a_tolerance)
{
  // How much time the vehicle needs to stop based on the acceleration
  linear_time_to_break_ = l_max_vel_ / l_max_acc_;
  angular_time_to_break_ = a_max_vel_ / a_max_acc_;

  // The formular velocity to time "f(time) = -accelration_max * time + velocity_max"
  // is integrated to get the surface of the function, which is the distance to break.
  linear_break_point_ = -(l_max_acc_/2) * pow(linear_time_to_break_, 2) + l_max_vel_ * linear_time_to_break_;
  angular_break_point_ = -(a_max_acc_/2) * pow(angular_time_to_break_, 2) + a_max_vel_ * angular_time_to_break_;

  linear_deacceleration_factor_ = l_max_vel_ / linear_break_point_;
  angular_deacceleration_factor_ = a_max_vel_ / angular_break_point_;
}

void OmniVelocityController::setTargetPose(const Pose& pose)
{
  target_pose_ = pose;
  linear_complete_ = false;
  angular_complete_ = false;
}

bool OmniVelocityController::isTargetReached() const
{
  return linear_complete_ & angular_complete_;
}

Velocity OmniVelocityController::computeVelocity(const Pose& actual_pose)
{
  // Displacement and orientation to the target in world frame
  double x_dist = target_pose_.x - actual_pose.x;
  double y_dist = target_pose_.y - actual_pose.y;

  // Step 1: compute remaining distances
  double linear_dist = getDistance(target_pose_, actual_pose);
  double angular_dist = getShortestAngle(target_pose_.theta, actual_pose.theta);

  if (std::abs(linear_dist) < l_tolerance_ && std::abs(angular_dist) < a_tolerance_)
  {
    linear_complete_ = true;
    angular_complete_ = true;
    return Velocity();
  }

  // Step 2: compute velocities
  double linear_vel = 0.0;
  double angular_vel = 0.0;

  // If the distance is bigger than the break point, than the vehicle drives with maximum speed,
  // else the speed is the distance multiplied with the deacceleration factor to reach the target smoothly.
  if (std::abs(linear_dist) > linear_break_point_) { linear_vel = l_max_vel_; }
  else { linear_vel = linear_dist * linear_deacceleration_factor_; }

  if (std::abs(angular_dist) > angular_break_point_) { angular_vel = a_max_vel_; }
  else { angular_vel = angular_dist * angular_deacceleration_factor_; }

  // Step 3: Divide linear velocity in velocities for x and y direction
  double x_ratio = x_dist / linear_dist;
  double y_ratio = y_dist / linear_dist;

  double x_vel = x_ratio * linear_vel;
  double y_vel = y_ratio * linear_vel;

  // Step 4: Velocities for X and Y direction are split to forward movement
  //         and sideward movement by rotating around theta.
  return Velocity(+ std::copysign(x_vel, x_dist) * cos(actual_pose.theta)
                  + std::copysign(y_vel, y_dist) * sin(actual_pose.theta), // Forward movement
                  - std::copysign(x_vel, x_dist) * sin(actual_pose.theta)
                  + std::copysign(y_vel, y_dist) * cos(actual_pose.theta), // Sideward movement
                  std::copysign(angular_vel, angular_dist)); // Angular movement
}

