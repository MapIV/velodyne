/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2018, Tier IV, Inc.
 *
 *  License: Modified BSD Software License Agreement
 *
 */

/*
This ROS node converts raw Velodyne LIDAR packets to PointCloud2 with timestamp.

Chenxi TU
*/

#include <ros/ros.h>
#include "velodyne_pointcloud/convert.h"

/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud_node_T");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  // create conversion class, which subscribes to raw data
  velodyne_pointcloud::Convert_T conv(node, priv_nh);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
