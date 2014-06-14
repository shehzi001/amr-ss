#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include <random>

#include "pose.h"

/** This class represents a motion model for omnidirectional robot and could be
  * used to sample the possible pose given the starting pose and the commanded
  * robot's motion.
  *
  * The two parameters of the class is standard deviations of translational and
  * rotational components of the motion.
  *
  * The motion is decomposed into two translations along the x axis of the
  * robot (forward), and along the y axis of the robot (lateral), and one
  * rotation.
  *
  * Usage:
  *
  * @code
  *   // Create motion model with 0.02 and 0.01 stddev
  *   MotionModel motion_model(0.02, 0.01);
  *   // Set the commanded robot's motion
  *   motion_model.setMotion(0.5, 0.1, 0.1);
  *   // Sample the possible pose given the starting pose
  *   // Note that it could be repeated multiple times for the same starting
  *   // pose of for different starting poses
  *   Pose new_pose = motion_model.sample(pose);
  * @code
  *
  * */
class MotionModel
{

public:

  MotionModel(double sigma_translation, double sigma_rotation)
  : forward_(0.0)
  , lateral_(0.0)
  , rotation_(0.0)
  , generator_(device_())
  , distribution_trans_(0, sigma_translation)
  , distribution_rot_(0, sigma_rotation)
  {
  }

  /** Set the commanded robot's motion. */
  void setMotion(double forward, double lateral, double rotation)
  {
    forward_ = forward;
    lateral_ = lateral;
    rotation_ = rotation;
  }

  /** Sample a possible pose resulting from the commanded robot's motion, if
    * the robot was in given pose. */
  Pose sample(const Pose& pose)
  {
    // Rotation of the motion
    double x_motion = forward_ * cos(-pose.theta) + lateral_ * sin(-pose.theta);
    double y_motion = -forward_ * sin(-pose.theta) + lateral_ * cos(-pose.theta);

    //Use forward, lateral, rotation to get desired motion delta_rot1, delta_trans, delta_rot2.(slide 25 localization 2)
    double delta_trans = sqrt(pow(x_motion, 2) + pow(y_motion, 2));
    double delta_rot1 = atan2(y_motion, x_motion);
    double delta_rot2 = rotation_ - delta_rot1;

    //Assign the noise distributions
    double trans_error = distribution_trans_(generator_);
    double rot1_error = distribution_rot_(generator_);
    double rot2_error = distribution_rot_(generator_);

    //Add noise
    double delta_trans_hat = delta_trans + trans_error;
    double delta_rot1_hat = delta_rot1 + rot1_error;
    double delta_rot2_hat = delta_rot2 + rot2_error;

    //Get the new pose
    Pose new_pose;
    new_pose.x = pose.x + delta_trans_hat * cos(rotation_ + delta_rot1_hat);
    new_pose.y = pose.y + delta_trans_hat * sin(rotation_ + delta_rot1_hat);
    new_pose.theta = normalizeAngle(pose.theta + delta_rot1_hat + delta_rot2_hat);

    return new_pose;
  }

private:

  inline double normalizeAngle(double angle)
  {
    return atan2(sin(angle), cos(angle));
  }

  double forward_;
  double lateral_;
  double rotation_;
  std::random_device device_;
  std::mt19937 generator_;
  std::normal_distribution<double> distribution_trans_;
  std::normal_distribution<double> distribution_rot_;

};

#endif /* MOTION_MODEL_H */

