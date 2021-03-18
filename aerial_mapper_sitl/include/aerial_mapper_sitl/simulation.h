#ifndef AERIAL_MAPPER_SITL_SIMULATION_H
#define AERIAL_MAPPER_SITL_SIMULATION_H

#include "ros/callback_queue.h"
#include <ros/ros.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>  // fromROSMsg
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>  // transformPointCloud
#include <sensor_msgs/PointCloud2.h>
#include <aerial-mapper-utils/utils-nearest-neighbor.h>
#include <mutex>

#include <aerial-mapper-dsm/dsm.h>
#include <aerial-mapper-grid-map/aerial-mapper-grid-map.h>
#include <aerial-mapper-io/aerial-mapper-io.h>
#include <aerial-mapper-utils/utils-nearest-neighbor.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

class Simulation {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Simulation(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  ~Simulation();

  void init();
  // void getPointClouds(AlignedType<std::vector, Eigen::Vector3d>::type* point_cloud);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher pointcloud_pub_;

  ros::Subscriber pointcloud_sub_;

  ros::Time last_time_;

  tf::TransformListener tf_listener_;

  ros::Timer cmdloop_timer_, statusloop_timer_;
  ros::CallbackQueue cmdloop_queue_, statusloop_queue_;
  std::unique_ptr<ros::AsyncSpinner> cmdloop_spinner_;
  std::unique_ptr<ros::AsyncSpinner> statusloop_spinner_;

  double cmdloop_dt_{0.1};
  double statusloop_dt_{1.0};

  AlignedType<std::vector, Eigen::Vector3d>::type point_cloud_;

  grid_map::Settings settings_aerial_grid_map_;
  dsm::Settings settings_dsm_;
  std::unique_ptr<grid_map::AerialGridMap> grid_map_;
  std::unique_ptr<dsm::Dsm> dsm_;

  std::recursive_mutex mutex_;

  /**
  * @brief     callaback for command loop timer
  * @param[in] even, ROS events
  **/
  void cmdLoopCallback(const ros::TimerEvent& event);

  /**
  * @brief     callaback for status loop timer
  * @param[in] even, ROS events
  **/
  void statusLoopCallback(const ros::TimerEvent& event);

  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
};
#endif  // AERIAL_MAPPER_SITL_SIMULATION_H
