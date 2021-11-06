/*
    DIAN Driverless.
    Copyright (c) 2021:
     - Sean Zhou <zhouxiao@dianracing.com>
*/

#ifndef LIVOX_CLUSTER_H
#define LIVOX_CLUSTER_H

#include "sensor_msgs/PointCloud2.h"
#include <chrono>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define range 20 //raw is 20
#define dx 0.2
#define dy 0.2
#define nx 101
#define ny 201
#define cone_height 0.3
#define cone_height_thre 0.1
#define inte_count 40
#define cluster_thre 0.1
#define cluster_min 5
#define cluster_max 10000

typedef pcl::PointXYZI point_type;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, 
    sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> sync_policy;


#endif // LIVOX_CLUSTER_H
