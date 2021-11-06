#pragma once

#include "ros/ros.h"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include "pcl/io/pcd_io.h"
#include <pcl/registration/icp.h>

#include "std_msgs/String.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"


#include "fstream"
#include "cmath"


namespace ns_skidpad_detector {

class SkidpadDetector
{
public:
	SkidpadDetector(ros::NodeHandle &nh);
	void loadParameters();
  	void subscribeToTopics();
  	void publishToTopics();
  	void clusterCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
  	void runAlgorithm();
  	void sendMsg();
  	void getTransMatrix();
private:
	ros::NodeHandle nh_;
	ros::Subscriber clusterFilteredSubscriber_;
	ros::Publisher transformMatrixPublisher_;
	ros::Publisher skidpadMapPublisher_;
	ros::Publisher matchCloudPublisher_;

	ros::Publisher relateCloudPublisher_;

	std::string cluster_filtered_topic_name_;
	std::string transform_matrix_topic_name_;

	std::string path_pcd_, path_x_, path_y_;
	double start_length_, lidar2imu_, threshold_;

	bool getClusterFlag, matchFlag;

	nav_msgs::Path trans_path, standard_path;

	pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr_ = 
		pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>); 

	pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_ptr_ = 
		pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>); //livox点云消息包含xyz和intensity 

	pcl::PointCloud<pcl::PointXYZ>::Ptr match_cloud_ptr_ = 
		pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr relate_cloud_ptr_ = 
		pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

	
  	Eigen::Matrix4f transformation;
  	std_msgs::Float64MultiArray trans_matrix_in_1D;

  private:
  	void loadFiles();


};

}// end of namespace ns_skidpad_detector