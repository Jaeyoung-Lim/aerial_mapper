/*
 *    Filename: simulation_node.cpp
 *  Created on: Mar 17, 2021
 *      Author: Jaeyoung Lim
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// NON-SYSTEM
#include <aerial-mapper-dense-pcl/stereo.h>
#include <aerial-mapper-dsm/dsm.h>
#include <aerial-mapper-grid-map/aerial-mapper-grid-map.h>
#include <aerial-mapper-io/aerial-mapper-io.h>
#include <aerial-mapper-utils/utils-nearest-neighbor.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "aerial_mapper_sitl/simulation.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "aerial_mapper_sitl");

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  Simulation simulation_node(nh, nh_private);
  simulation_node.init();
  ros::spin();

  return 0;
}
