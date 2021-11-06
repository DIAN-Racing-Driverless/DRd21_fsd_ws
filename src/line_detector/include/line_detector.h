#pragma once

#include "ros/ros.h"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "nav_msgs/Path.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace ns_line_detector {
class LineDetector
{
public:
	LineDetector(ros::NodeHandle& nh);
	void runAlgorithm();
	void loadParameters();
	void subscribeToTopics();
	void lidarClusterCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
	void publishToTopics();
	void sendMsg();

	void findNextPoint(const geometry_msgs::Point &OriginalPoint, std::vector<geometry_msgs::Point> &BorderPoints, const pcl::PointCloud<pcl::PointXYZI>::Ptr &clusteredPC);
	void clustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr &conesPC, pcl::PointCloud<pcl::PointXYZI>::Ptr &clusteredPC);
	void LineFitLeastSquares(std::vector<geometry_msgs::Point> MiddlePoints, std::vector<double> &vResult);
	void visualization_path(std::vector<geometry_msgs::Point> MiddlePoints);

	int laser_count;
	pcl::PointCloud<pcl::PointXYZI>::Ptr conesPC = 
		pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
	

private:

	ros::NodeHandle nh_;

	ros::Subscriber lidarClusterSubscriber_;
  	ros::Publisher endPointPublisher_;
	ros::Publisher PathPublisher_;
	ros::Publisher MiddlePublisher_;
	ros::Publisher vis_pub;

	std::string lidar_cluster_topic_name_;  
	std::string end_point_topic_name_;	

	pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_ptr_ = 
		pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>); //livox点云消息包含xyz和intensity 
	
	geometry_msgs::Point end_point_;

	bool getPath = false;
	double path_length;
	double allow_angle_error;

	void createPath();
};

} // end of namespace ns_line_detector