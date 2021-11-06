//ReCode by HoGinHang on Fri Nov 5
/*
TODO:
ICP配准失效主要是输入的点cloud_in与cloud_out两两不对应导致的，
这种情况的原因，一方面是本身测试环境的锥筒摆放与预设锥筒位置不同，导致ICP点在找对应关系时匹配错误；
另一方面是对应过程中，还会有部分预设点与感知到的锥筒点不识别的BUG
*/

#include "ros/ros.h"
#include "skidpad_detector.h"

namespace ns_skidpad_detector
{
SkidpadDetector::SkidpadDetector(ros::NodeHandle &nh)
{
	nh_=nh;
	loadParameters();
	subscribeToTopics();
	publishToTopics();
	loadFiles();
	matchFlag=false;
}

void SkidpadDetector::loadParameters()
{
	ROS_INFO("loading handle parameters");
	if (!nh_.param<std::string>("/planning/cluster_filtered_topic_name",
	                                  cluster_filtered_topic_name_,
	                                  "/perception/lidar_cluster")) {
	ROS_WARN_STREAM("Did not load cluster_filtered_topic_name. Standard value is: " << cluster_filtered_topic_name_);
	}
	if (!nh_.param<std::string>("/planning/transform_matrix_topic_name",
	                                  transform_matrix_topic_name_,
	                                  "/planning/transform_matrix")) {
	ROS_WARN_STREAM("Did not load transform_matrix_topic_name. Standard value is: " << transform_matrix_topic_name_);
	}
	if (!nh_.param<std::string>("/planning/path/skidpad_map",
	                                  path_pcd_,
	                                  "skidpad_detector/skidpad.pcd")) {
	ROS_WARN_STREAM("Did not load path/skidpad_map. Standard value is: " << path_pcd_);
	}

		if (!nh_.param<std::string>("/planning/path/path_x",
	                                  path_x_,
	                                  "skidpad_detector/path_x.txt")) {
	ROS_WARN_STREAM("Did not load path/path_x. Standard value is: " << path_x_);
	}

	if (!nh_.param<std::string>("/planning/path/path_y",
	                                  path_y_,
	                                  "skidpad_detector/path_y.txt")) {
	ROS_WARN_STREAM("Did not load path/path_y. Standard value is: " << path_y_);
	}

	if (!nh_.param("/planning/length/start", start_length_, 15.0)) {
	ROS_WARN_STREAM("Did not load start_length. Standard value is: " << start_length_);
	}

	if (!nh_.param("/planning/length/lidar2imu", lidar2imu_, 0.0)) {
	ROS_WARN_STREAM("Did not load lidar2imu. Standard value is: " << lidar2imu_);
	}

	if (!nh_.param("/planning/length/threshold", threshold_, 0.7)) {
	ROS_WARN_STREAM("Did not load length/threshold. Standard value is: " << threshold_);
	}


}
void SkidpadDetector::subscribeToTopics() {
	ROS_INFO("subscribe to topics");
	clusterFilteredSubscriber_ = nh_.subscribe(cluster_filtered_topic_name_, 1, &SkidpadDetector::clusterCallback, this);
}

void SkidpadDetector::publishToTopics()
{
	ROS_INFO("publish to topics");
  	transformMatrixPublisher_ = nh_.advertise<std_msgs::Float64MultiArray>(transform_matrix_topic_name_, 1);
  	skidpadMapPublisher_ = nh_.advertise<sensor_msgs::PointCloud2>("planning/map_cloud",3);
  	matchCloudPublisher_ = nh_.advertise<sensor_msgs::PointCloud2>("planning/match_cloud",3);
	relateCloudPublisher_ = nh_.advertise<sensor_msgs::PointCloud2>("planning/relate_cloud",3);
}

// Getters
void SkidpadDetector::getTransMatrix() {
  trans_matrix_in_1D.data.clear();
  for (int i = 0; i < transformation.rows(); i++)
    for (int j = 0; j < transformation.cols(); j++) {
      trans_matrix_in_1D.data.push_back(transformation(i, j));
      //std::cout<<transformation(i, j)<<" ";
    }
    //std::cout<<std::endl;
}

void SkidpadDetector::sendMsg()
{
	getTransMatrix();
  	transformMatrixPublisher_.publish(trans_matrix_in_1D);

  	//publish map cloud
  	sensor_msgs::PointCloud2 map_msg;
  	pcl::toROSMsg(*map_ptr_,map_msg);
  	map_msg.header.frame_id="livox_frame";
  	map_msg.header.stamp = ros::Time::now();
  	skidpadMapPublisher_.publish(map_msg);
	
  	//publish match cloud
  	sensor_msgs::PointCloud2 match_msg;
  	pcl::toROSMsg(*match_cloud_ptr_,match_msg);
  	match_msg.header.frame_id="livox_frame";
  	match_msg.header.stamp = ros::Time::now();
  	matchCloudPublisher_.publish(match_msg);


  	//publish relate cloud
  	sensor_msgs::PointCloud2 relate_msg;
  	pcl::toROSMsg(*relate_cloud_ptr_,relate_msg);
  	relate_msg.header.frame_id="livox_frame";
  	relate_msg.header.stamp = ros::Time::now();
	relateCloudPublisher_.publish(relate_msg);
}

void SkidpadDetector::clusterCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	getClusterFlag = true;
	pcl::fromROSMsg(*msg,*cluster_ptr_);
}

void SkidpadDetector::loadFiles()
{
	pcl::PointCloud<pcl::PointXYZ> source_cloud;
	pcl::PointXYZ tmp_cloud;
	pcl::io::loadPCDFile (path_pcd_,source_cloud);
	ROS_INFO_STREAM("load files");
	ROS_INFO("source size is %ld",source_cloud.points.size());
	for(int i = 0; i < source_cloud.points.size(); i++)
	{
		tmp_cloud.x = source_cloud.points[i].y + start_length_ + lidar2imu_;
		tmp_cloud.y = -source_cloud.points[i].x;
		map_ptr_->points.push_back(tmp_cloud);

	}
	/* load skidpad path */
	std::ifstream infile_x,infile_y;
	infile_x.open(path_x_);
	infile_y.open(path_y_);
	double path_x,path_y;

	while(!infile_x.eof() && !infile_y.eof())
	{
		infile_x>>path_x;
		infile_y>>path_y;
		geometry_msgs::PoseStamped temp;
		temp.pose.position.x = path_x + lidar2imu_;
		temp.pose.position.y = path_y;
		standard_path.poses.push_back(temp);
	}
	infile_x.close();
	infile_y.close();
}

void SkidpadDetector::runAlgorithm()
{
  if(!getClusterFlag)
	{
		ROS_INFO("don't get the cluster point");
		return;
	}
	if(matchFlag)
		return;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> Final;
	pcl::PointXYZ in_temp;
	pcl::PointXYZ out_temp;

	int cluster_points = cluster_ptr_->points.size();
	ROS_INFO("we have %d cluster points", cluster_points);

	// find match points between skidpad map and cluster, match distance < threshold_
	std::vector<int> index_vector_;
	for(int i=0;i<map_ptr_->points.size();i++)
	{
		// double min_dist = std::numeric_limits<double>::infinity();
		double min_dist = 999;
		int index = -1;
		int index_check = -1;
		bool isTheSame = false;

		//search for the nearest-point
		for(int j=0;j<cluster_ptr_->points.size();j++)
		{
			double dist = std::hypot(map_ptr_->points[i].x - cluster_ptr_->points[j].x, map_ptr_->points[i].y - cluster_ptr_->points[j].y);
			if(min_dist>dist)
			{
				min_dist = dist;
				index_check = j;
			}
		}
		
		//check the same point, but here we still have bugs
		if(index_vector_.size()==0){
			index_vector_.push_back(index_check);
			index = index_check;
		}
		else{
			for(auto &iter: index_vector_){
				if(iter == index_check){
					isTheSame = true;
				}
			}
			if(!isTheSame){
				index_vector_.push_back(index_check);
				index = index_check;
			}
		}
		
		//check nearest-point, and then push recent related_points, bugs maybe occur here
		if(min_dist<threshold_ && index >= 0)
		{
			in_temp.x = map_ptr_->points[i].x;
			in_temp.y = map_ptr_->points[i].y;
			in_temp.z = 0;
			cloud_in->points.push_back(in_temp);

			out_temp.x = cluster_ptr_->points[index].x;
			out_temp.y = cluster_ptr_->points[index].y;
			out_temp.z = 0;
			cloud_out->points.push_back(out_temp);
		}
	}
	ROS_INFO("cloud_in size is %ld",cloud_in->points.size());
	ROS_INFO("cloud_out size is %ld",cloud_out->points.size());
	
	if(cloud_out->points.size()>8)
	{
		match_cloud_ptr_=cloud_out;
		relate_cloud_ptr_=cloud_in;
	  // icp match
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setInputSource(cloud_in);
		icp.setInputTarget(cloud_out);
		icp.setMaxCorrespondenceDistance(3);
		icp.setTransformationEpsilon(1e-10);   
		icp.setEuclideanFitnessEpsilon(0.0001); 
		icp.setMaximumIterations(1000);//100
		icp.align(Final);
		ROS_INFO_STREAM("icp finish");

		std::cout << "the icp has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
		transformation = icp.getFinalTransformation();
		std::cout <<"------------------------------------------------\n";
		std::cout <<transformation<<std::endl;
		std::cout <<"------------------------------------------------\n";
		matchFlag = true;
	}
}

}


	