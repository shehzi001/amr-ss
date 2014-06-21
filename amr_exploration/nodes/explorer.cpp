#include <ros/ros.h>
#include <ros/console.h>

#include <amr_msgs/ExecutePathGoal.h>
#include <amr_msgs/ExecutePathAction.h>
#include <amr_msgs/ExecutePathActionResult.h>
#include <amr_srvs/PlanPath.h>

#include <amr_msgs/PathExecutionFailure.h>
#include <amr_msgs/ExecutePathGoal.h>
#include <amr_msgs/ExecutePathAction.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <amr_srvs/GetNearestOccupiedPointOnBeam.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <math.h>
#include <tf/tf.h>
#include <ros/duration.h>

#include "clustered_point_cloud_visualizer.h"
#include "frontier.h"

#define mapCallback_debug false
#define explore_debug true
#define robot_position_debug false
#define move_point_near_debug false

#define reduction 0.1

class ExplorerNode
{

public:

  ExplorerNode()
  : frontier_clusters_publisher_("frontier_clusters", "odom")
  , world_is_explored_(false)
  , position_reported(false)
  , path_exec_("/path_executor/execute_path",true)
  // , plan_nh_("~")
  {
    frontier_publisher_ = nh_.advertise<Frontier::PointCloud>("frontier_points", 1);
    map_subscriber_ = nh_.subscribe("sonar_mapper/map", 1, &ExplorerNode::mapCallback, this);
    position_subscriber_ = nh_.subscribe("/odom", 1, &ExplorerNode::robot_position_cb, this);

    robot_position_.position.x = 0;
    robot_position_.position.y = 0;
    path_planer_ = nh_.serviceClient<amr_srvs::PlanPath>("path_planner/plan_path");
    occupancy_query_ = occupancy_nh_.serviceClient<amr_srvs::GetNearestOccupiedPointOnBeam>("/occupancy_query_server/get_nearest_occupied_point_on_beam");

    ROS_INFO_STREAM("Wating for path executor...");
    path_exec_.waitForServer();
  }

  void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
  {
    Frontier frontier(msg);
    frontier_publisher_.publish(frontier.getPointCloud());
    frontier_clusters_publisher_.publish<Frontier::Point>(frontier.getClusterPointClouds());

    std::vector<Frontier::PointCloud::Ptr> frontier_clusters = frontier.getClusterPointClouds();
    
    Frontier::PointCloud::VectorType frontiers_center = frontier.getClusterCentroids();
    frontier_clusters_centroids_.resize(frontiers_center.size());

    ROS_WARN_STREAM_COND(mapCallback_debug,"Frointer Count :"<<frontier_clusters.size());
    int j =0;
    for(int i = 0;i < frontier_clusters.size(); i++){
      Frontier::PointCloud::Ptr cluster = frontier_clusters[i];
      if(cluster->size() > 10){
        frontier_clusters_centroids_[j] = frontiers_center[i];
        j = j +1;
      }
    }
    frontier_clusters_centroids_.resize(j);
    if(frontier_clusters_centroids_.size() > 0){
      world_is_explored_ = false;
    }else{
      world_is_explored_ = true;
    }
    ROS_INFO_STREAM_COND(mapCallback_debug,"Eligible Frontier Count: "<<frontier_clusters_centroids_.size());
  }

  void explore()
  {
    while (ros::ok() && !world_is_explored_)
    {
      bool path_planed = false;
      // do not start till you have a frontier
      if(frontier_clusters_centroids_.size() <= 0 || !position_reported){
      }else{
        // Choosing a target
        // the frontier with least distance form robot.
        int ind = 0;
        int plan_ind = 0;
        amr_srvs::PlanPath srv;
        for(int i=0; i < frontier_clusters_centroids_.size(); i++){
          Frontier::Point point = frontier_clusters_centroids_[i];
          if(!path_planed && plan_path_to(point,srv)){
            path_planed = true;
            plan_ind = i;
          }
          if(i != 0 && distance_from_robot(point) < distance_from_robot(frontier_clusters_centroids_[ind])){
            ind = i;
          }
        }
        Frontier::Point new_point;
        if(!path_planed){
          ROS_INFO_STREAM_COND(explore_debug,"Found Nearest Frontier at index:"<<ind<<" at distance"<<distance_from_robot(frontier_clusters_centroids_[ind]));
          new_point = move_point_near_robot(frontier_clusters_centroids_[ind]);
          // Planing a path to current target
          path_planed = plan_path_to(new_point,srv);
        }
        
        // Executeing the path
        amr_msgs::ExecutePathGoal goal;
        goal.skip_unreachable = true;
        ros::Duration wait_duration;
        if(path_planed){
          goal.path.poses.resize(srv.response.path.poses.size());
          goal.path.poses = srv.response.path.poses;
          wait_duration = ros::Duration(0,0);
        }else{
          goal.path.poses.resize(1);
          geometry_msgs::PoseStamped pose_s;
          pose_s.pose.position.x = new_point.x;
          pose_s.pose.position.y = new_point.y;
          pose_s.pose.orientation = robot_position_.orientation;
          goal.path.poses[0] = pose_s;
          wait_duration= ros::Duration(0,0);
        }
        path_exec_.sendGoal(goal);
        if(!path_exec_.waitForResult(wait_duration)){
          ROS_ERROR("Path Not Completed");
        }
        
        // ...
      }
      ros::spinOnce();
    }
    ROS_INFO_COND(world_is_explored_, "World is completely explored, exiting...");
  }

  void robot_position_cb(const nav_msgs::Odometry& msg){

    robot_position_.position.x = msg.pose.pose.position.x;
    robot_position_.position.y = msg.pose.pose.position.y;
    robot_position_.orientation = msg.pose.pose.orientation;
    position_reported = true;
    ROS_INFO_STREAM_COND(robot_position_debug,"Robot is at: ("<<robot_position_.position.x<<","<<robot_position_.position.y<<")");
  }

  Frontier::Point move_point_near_robot(Frontier::Point point){
      ROS_INFO_COND(move_point_near_debug,"Moving Point(%2f,%2f)",point.x,point.y);
      amr_srvs::GetNearestOccupiedPointOnBeam service;
      service.request.beams.resize(1);
      service.request.beams[0].x = point.x;
      service.request.beams[0].y = point.y;
      service.request.beams[0].theta = atan2(point.y- robot_position_.position.y,point.x- robot_position_.position.x);
      if (occupancy_query_.call(service)){
        ROS_INFO_STREAM_COND(explore_debug,"nerest points count "<<service.response.points.size());

      }else{
        ROS_ERROR("Failed to call service occupancy_query_server");
      }
      Frontier::Point new_point;
      new_point.x= service.response.points[0].x;
      new_point.y= service.response.points[0].y;
      ROS_INFO_COND(move_point_near_debug,"New Point(%2f,%2f)",new_point.x,new_point.y);
      return new_point;
  }

  float distance_from_robot(Frontier::Point point){
    //Getting Robot position
    
    return sqrt(pow(robot_position_.position.x - point.x,2)+pow(robot_position_.position.y - point.y,2));
  }

  bool plan_path_to(Frontier::Point end_point,amr_srvs::PlanPath& srv){
    // Planing a path to current target
    srv.request.start.x = robot_position_.position.x;
    srv.request.start.y = robot_position_.position.y;
    srv.request.end.x = end_point.x;
    srv.request.end.y = end_point.y;
    ROS_INFO_STREAM_COND(explore_debug,"Planing Path to Nearest Frontier...");
    ROS_INFO_STREAM_COND(explore_debug,"From: ("<<srv.request.start.x<<","<<srv.request.start.y<<") To: ("<<srv.request.end.x<<","<<srv.request.end.y<<")");
    if (path_planer_.call(srv)){
      ROS_INFO_STREAM_COND(explore_debug,"Path Planed with "<<srv.response.path.poses.size()<<" poses");
      return true;
    }else{
      ROS_ERROR("Failed to call service plan_path");
      return false;
    }
  }

private:

  ros::NodeHandle nh_;
  ros::NodeHandle occupancy_nh_;

  ros::Subscriber map_subscriber_;
  ros::Publisher frontier_publisher_;
  ClusteredPointCloudVisualizer frontier_clusters_publisher_;

  ros::Subscriber position_subscriber_;
  ros::ServiceClient path_planer_;
  ros::ServiceClient occupancy_query_;
  actionlib::SimpleActionClient<amr_msgs::ExecutePathAction> path_exec_;
  Frontier::PointCloud::VectorType frontier_clusters_centroids_;
  bool world_is_explored_;
  bool position_reported;
  geometry_msgs::Pose robot_position_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "explorer");
  ExplorerNode en;
  en.explore();
  return 0;
}
