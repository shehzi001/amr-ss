#ifndef OMNI_VELOCITY_CONTROLLER_H
#define OMNI_VELOCITY_CONTROLLER_H

#include "velocity_controller.h"

/** An implementation of velocity controller that moves the robot as if it had
  * an omni-directional base. */
class OmniVelocityController : public VelocityController
{

public:

  OmniVelocityController(double l_max_vel, double l_max_acc, double l_tolerance,
                         double a_max_vel, double a_max_acc, double a_tolerance);

  virtual void setTargetPose(const Pose& pose);

  virtual bool isTargetReached() const;

  virtual Velocity computeVelocity(const Pose& actual_pose);

private:

  Pose target_pose_;

  bool linear_complete_;
  bool angular_complete_;

  double l_max_vel_;
  double l_max_acc_;
  double l_tolerance_;

  double a_max_vel_;
  double a_max_acc_;
  double a_tolerance_;

  double linear_time_to_break_;
  double angular_time_to_break_;

  double linear_break_point_;
  double angular_break_point_;

  double linear_deacceleration_factor_;
  double angular_deacceleration_factor_;

};

#endif /* OMNI_VELOCITY_CONTROLLER_H */

