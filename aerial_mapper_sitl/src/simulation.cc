
#include "aerial_mapper_sitl/simulation.h"
#include <tf_conversions/tf_eigen.h>

Simulation::Simulation(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {

  settings_aerial_grid_map_.center_easting = 0.0;
  settings_aerial_grid_map_.center_northing = 0.0;
  settings_aerial_grid_map_.delta_easting = 200.0;
  settings_aerial_grid_map_.delta_northing = 200.0;
  settings_aerial_grid_map_.resolution = 0.5;
  grid_map_ = std::make_unique<grid_map::AerialGridMap>(settings_aerial_grid_map_);

  LOG(INFO) << "Create DSM (batch).";
  settings_dsm_.center_easting = settings_aerial_grid_map_.center_easting;
  settings_dsm_.center_northing = settings_aerial_grid_map_.center_northing;
  dsm_ = std::make_unique<dsm::Dsm>(settings_dsm_, grid_map_->getMutable());
}

Simulation::~Simulation() {}

void Simulation::init() {
  last_time_ = ros::Time::now();
  ros::TimerOptions cmdlooptimer_options(ros::Duration(cmdloop_dt_),
                                         boost::bind(&Simulation::cmdLoopCallback, this, _1), &cmdloop_queue_);
  cmdloop_timer_ = nh_.createTimer(cmdlooptimer_options);

  ros::TimerOptions statuslooptimer_options(
      ros::Duration(statusloop_dt_), boost::bind(&Simulation::statusLoopCallback, this, _1), &statusloop_queue_);
  statusloop_timer_ = nh_.createTimer(statuslooptimer_options);

  pointcloud_sub_ = nh_.subscribe("/camera/depth/points", 1, &Simulation::pointCloudCallback, this);
  pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/camera/depth/transformed_points", 1);

  cmdloop_spinner_.reset(new ros::AsyncSpinner(1, &cmdloop_queue_));
  cmdloop_spinner_->start();
  statusloop_spinner_.reset(new ros::AsyncSpinner(1, &statusloop_queue_));
  statusloop_spinner_->start();
}

void Simulation::cmdLoopCallback(const ros::TimerEvent& event) {}

void Simulation::statusLoopCallback(const ros::TimerEvent& event) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  if (point_cloud_.empty()) {
    return;
  }
  dsm_->process(point_cloud_, grid_map_->getMutable());
  grid_map_->publishOnce();

  std::cout << "Cleared pointclouds" << std::endl;
  point_cloud_.clear();
}

void Simulation::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  ros::Time current_time = ros::Time::now();
  if ((current_time - last_time_).toSec() < 0.05) {
    return;
  }

  //Transform the pointcloud into world frame
  pcl::PointCloud<pcl::PointXYZ> untransformed_cloud;
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud;

  //First, lookup the transform
  tf::StampedTransform transform;
  try{
    tf_listener_.lookupTransform( "/local_origin", "/camera_link", ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return;
  }

  Eigen::Affine3d transform_1;
  tf::poseTFToEigen(transform, transform_1);

  pcl::fromROSMsg(*msg, untransformed_cloud);

  //Second, transform the pointcloud into the world frame
  pcl::transformPointCloud(untransformed_cloud, transformed_cloud, transform_1);

  //Verify that the tranform is correct
  sensor_msgs::PointCloud2 transformed_pt_msg;
  transformed_pt_msg.header.stamp = ros::Time::now();
  pcl::toROSMsg(transformed_cloud, transformed_pt_msg);
  transformed_pt_msg.header.frame_id = "local_origin";
  pointcloud_pub_.publish(transformed_pt_msg);

  //TODO: Accumulate pointclouds here
  for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = transformed_cloud.begin();
   it != transformed_cloud.end(); ++it) {
    Eigen::Vector3d point(it->x, it->y, it->z);
    point_cloud_.push_back(point);
  }
  last_time_ = current_time;
}
