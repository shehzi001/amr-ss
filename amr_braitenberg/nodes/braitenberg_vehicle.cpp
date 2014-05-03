/** Description of behavior:
  *
  * Braitenberg Type A:
  * In this mode the vehicle tries not to crush into obstacles. The vehicle dodges sideways.
  * If the transmission factor is to small the vehicle is slow, can not make fast turns and can still crash.
  * If it is to high and the vehicle is driving between obstacles the vehicle the rotation angle oscillates.
  * And if the factor is negative, the vehicle drives perpendicular away from the obstacle.
  *
  * Braitenberg Type B:
  * A Braitenberg type B  tries to reach a light source. In our case that is an obstacle and so the vehicle
  * drives perpendicular to the object and crashes. If the transmission factor is negative, the vehicle drives
  * sideways away from an obstacle.
  *
  * Braitenberg Type C:
  * In this combined mode the turn behaviors of the type A and type B vehicle cancel each other out, but the
  * speed is combined. So if the transmission factor of the type A behavior is higher than the transmission
  * factor for type B than the vehicle tries not to crash into an obstacle. The difference to a pure type
  * A vehicle is than, that it's speed is higher but the turn speed is damped. If the transmission factor of
  * the type B behavior is higher, than the vehicle crashes. If the factors are negative, than the vehicle
  * drives away from the obstacle. And depending on which type has the lowest transmission, the vehicle drives
  * sideways or perpendicular away. */


#include <ros/ros.h>
#include <ros/console.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>

#include <amr_msgs/Ranges.h>
#include <amr_msgs/WheelSpeeds.h>
#include <amr_srvs/SwitchRanger.h>
#include <amr_braitenberg/BraitenbergVehicleConfig.h>
#include "braitenberg_vehicle.h"

ros::Subscriber sonar_subscriber;
ros::Publisher wheel_speeds_publisher;
BraitenbergVehicle::UPtr vehicle;

/** Reconfiguration callback is triggered every time the user changes some
  * field through the rqt_reconfigure interface. */
void reconfigureCallback(amr_braitenberg::BraitenbergVehicleConfig &config, uint32_t level)
{
  /** To reconfigure a new braitenberg vehicle is created and appointed.
    * Because the config.type is an int value it is beeing casted to the braitenberg vehicle type. */
  vehicle = BraitenbergVehicle::UPtr(new BraitenbergVehicle(static_cast<BraitenbergVehicle::Type>(config.type),
                                                            config.factor1,
                                                            config.factor2));

  ROS_INFO("Vehicle reconfigured: type %i, factors %.2f and %.2f", config.type, config.factor1, config.factor2);
}

/** Sonar callback is triggered every time the Stage node publishes new data
  * to the sonar topic. */
void sonarCallback(const amr_msgs::Ranges::ConstPtr& msg)
{
  amr_msgs::WheelSpeeds m;

  /** To calculate the wheel speeds, the output from the lasers and the speeds are given to the
    * computeWheelSpeeds methods. Because WheelSpeeds is a vector with zero elemnts it's resized
    * to store two speeds for the left and right wheel. The handover of the speeds is done by
    * call-by-reference, so that a return is not needed. Values from the sensors are beeing
    * normalized here as well. */
  m.speeds.resize(2);
  vehicle->computeWheelSpeeds(msg->ranges[0].range / msg->ranges[0].max_range, msg->ranges[1].range / msg->ranges[1].max_range, m.speeds.at(0), m.speeds.at(1));

  wheel_speeds_publisher.publish(m);
  ROS_DEBUG("[%.2f %.2f] --> [%.2f %.2f]", msg->ranges[0].range, msg->ranges[1].range, m.speeds[0], m.speeds[1]);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "braitenberg_vehicle");
  ros::NodeHandle nh;
  // Wait until SwitchRanger service (and hence stage node) becomes available.
  ROS_INFO("Waiting for the /switch_ranger service to be advertised...");
  ros::ServiceClient switch_ranger_client = nh.serviceClient<amr_srvs::SwitchRanger>("/switch_ranger");
  switch_ranger_client.waitForExistence();
  // Make sure that the braitenberg sonars are available and enable them.
  amr_srvs::SwitchRanger srv;
  srv.request.name = "sonar_braitenberg";
  srv.request.state = true;
  if (switch_ranger_client.call(srv))
  {
    ROS_INFO("Enabled braitenberg sonars.");
  }
  else
  {
    ROS_ERROR("Braitenberg sonars are not available, shutting down.");
    return 1;
  }
  // Create default vehicle.
  vehicle = BraitenbergVehicle::UPtr(new BraitenbergVehicle);
  // Create subscriber and publisher.
  sonar_subscriber = nh.subscribe("/sonar_braitenberg", 100, sonarCallback);
  wheel_speeds_publisher = nh.advertise<amr_msgs::WheelSpeeds>("/cmd_vel_diff", 100);
  // Create dynamic reconfigure server.
  dynamic_reconfigure::Server<amr_braitenberg::BraitenbergVehicleConfig> server;
  server.setCallback(boost::bind(&reconfigureCallback, _1, _2));
  // Start infinite loop.
  ROS_INFO("Started braitenberg vehicle node.");
  ros::spin();
  return 0;
}

