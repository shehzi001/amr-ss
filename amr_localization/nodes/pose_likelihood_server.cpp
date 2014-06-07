#define _USE_MATH_DEFINES
#include <cmath>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <amr_srvs/GetPoseLikelihood.h>
#include <amr_srvs/GetNearestOccupiedPointOnBeam.h>
#include <amr_srvs/SwitchRanger.h>

class PoseLikelihoodServerNode
{

public:

  // Constructor
  PoseLikelihoodServerNode()
  {
    ros::NodeHandle pn("~");
    laserscan_subscriber_ = nh_.subscribe("/scan_front", 1, &PoseLikelihoodServerNode::laserCallback, this);
    beam_client_ = nh_.serviceClient<amr_srvs::GetNearestOccupiedPointOnBeam> ("/occupancy_query_server/get_nearest_occupied_point_on_beam");
    likelihood_server_ = pn.advertiseService("/pose_likelihood_server/get_pose_likelihood", &PoseLikelihoodServerNode::likelihoodCallback, this);
    ROS_INFO("Started [pose_likelihood_server] node.");
    getLaserTransform();
  }

  // Step 1: Get position of laser in respect to the base
  void getLaserTransform()
  {
    try
    {
      listener.waitForTransform("/base_link", "/base_laser_front_link", ros::Time(0), ros::Duration(2.0));
      listener.lookupTransform("/base_link", "/base_laser_front_link", ros::Time(0), laser_transform);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("Transform lookup failed: %s", ex.what());
    }
  }

  // Step 2: Get the sensed laser scans and store them.
  void laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
  {
    // Gets parameters of the laser
    range_max = msg->range_max;
    angle_min = msg->angle_min;
    angle_increment = msg->angle_increment;
    no_of_lasers = lround(std::abs(msg->angle_max - angle_min) / angle_increment) + 1;

    // Saves the all real sensed readings as z_r
    for (int i = 0; i < no_of_lasers; i++)
    {
      z_r[i] = msg->ranges[i];
    }
  }

  // Step 3: Implement likelihood callback
  bool likelihoodCallback(amr_srvs::GetPoseLikelihood::Request& request, amr_srvs::GetPoseLikelihood::Response& response)
  {
    // This datatype is passed around in ROS messages and is just a data storage with no associated functions.
    geometry_msgs::Pose pose_msg;
    pose_msg = request.pose.pose;
    // Convert pose message to transform
    tf::Transform pose_tf;
    tf::poseMsgToTF(pose_msg, pose_tf);
    // Send the request to OQS.
    amr_srvs::GetNearestOccupiedPointOnBeam srv_pointonbeam;
    std::vector<geometry_msgs::Pose2D> beams_poses;
    beams_poses.resize(16);

    for (int i = 0; i < no_of_lasers; i++)
    {
      query_input = pose_tf * laser_transform;// Transforming beam_pose from base link to odom
      // Extracting beam_pose x,y,theta w.r.t odom
      beams_poses[i].x = query_input.getOrigin().getX();
      beams_poses[i].y = query_input.getOrigin().getY();
      tf::Quaternion quaternion = query_input.getRotation();
      beams_poses[i].theta = tf::getYaw(quaternion) + angle_min + (angle_increment * i); // Calculates theta of laser sample
    }

    srv_pointonbeam.request.beams = beams_poses;
    srv_pointonbeam.request.threshold = 50.0;// Changing this changes the no. of red squares.

    if (beam_client_.call(srv_pointonbeam))
    {
      double sigma = 0.5;
      double w = 0.0;
      double sum = 0.0;
      double average_prob = 0.0;

      for (int i = 0; i < no_of_lasers; i++)
      {
        z_f[i] = srv_pointonbeam.response.distances[i]; // Fake distances

        // Clamping distances
        if (z_f[i] > range_max) { z_f[i] = range_max; }
        else if (z_f[i] < 0.0) { z_f[i] = 0.0; }

        // Compute the probabilities
        w = (1.0 / (sigma * sqrt(2.0 * M_PI))) * (exp(-pow((z_f[i] - z_r[i]), 2) / (2.0 * pow(sigma, 2))));

        sum = sum + w;
      }

      average_prob = sum / no_of_lasers;

      // Clamping probablities
      if (average_prob > 1.0) { average_prob = 1.0; }
      else if (average_prob < 0.0) { average_prob = 0.0; }

      // Respond as likelihood
      response.likelihood = average_prob;

      return true;
    }
    else
    {
      ROS_WARN("Server get nearest occupied point on the beam failed.");
      return false;
    }
  }

private:

  ros::NodeHandle nh_;
  ros::Subscriber laserscan_subscriber_;
  ros::ServiceServer likelihood_server_;
  ros::ServiceClient beam_client_;
  int no_of_lasers;
  double range_max;
  double angle_min;
  double angle_increment;
  double z_r[16];
  double z_f[16];
  tf::TransformListener listener;
  tf::StampedTransform sensor_transforms[16];
  tf::StampedTransform laser_transform;
  tf::Transform query_input;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_likelihood_server");
  ros::NodeHandle nh;
  // Wait until SwitchRanger service (and hence stage node) becomes available.
  ROS_INFO("Waiting for the /switch_ranger service to be advertised...");
  ros::ServiceClient switch_ranger_client = nh.serviceClient<amr_srvs::SwitchRanger> ("/switch_ranger");
  switch_ranger_client.waitForExistence();
  // Make sure that the hokuyo laser is available and enable them.
  amr_srvs::SwitchRanger srv;
  srv.request.name = "scan_front";
  srv.request.state = true;
  if (switch_ranger_client.call(srv))
  {
    ROS_INFO("Enabled hokuyo laser.");
  }
  else
  {
    ROS_ERROR("Hokuyo laser is not available, shutting down.");
    return 1;
  }
  PoseLikelihoodServerNode plsn;
  ros::spin();
  return 0;
}

