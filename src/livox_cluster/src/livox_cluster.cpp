/*
    DIAN Driverless.
    Copyright (c) 2021:
     - Sean Zhou <zhouxiao@dianracing.com>
*/

#include "livox_cluster.h"

ros::Publisher out_pub, ROI_pub, inte_pub, filter_pub, vis_pub;
pcl::PointCloud<point_type>::Ptr intePC(new pcl::PointCloud<point_type>);
uint16_t integral_count;
bool inte_flag;

/*  
    ---------------------------------------------------------------------------
                          EUCLIDEAN CLUSTERING MODULE
    ---------------------------------------------------------------------------
*/  
void clustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr &conesPC, 
	pcl::PointCloud<point_type>::Ptr &clusteredPC, double threshold, uint16_t min, uint16_t max) {

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
	tree->setInputCloud(conesPC);

	std::vector<pcl::PointIndices> clusterIndices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
	ec.setClusterTolerance(threshold);
	ec.setMinClusterSize(min);
	ec.setMaxClusterSize(max);
	ec.setSearchMethod(tree);
	ec.setInputCloud(conesPC);
	ec.extract(clusterIndices);

	visualization_msgs::MarkerArray bounding_boxes;
	int id_count = 0;
	// for(const auto &iter : iterator) -> read only
	for (const auto &iter : clusterIndices) {
		pcl::PointCloud<pcl::PointXYZI>::Ptr cones(new pcl::PointCloud<pcl::PointXYZI>);
		for (auto it : iter.indices) {
			cones->points.push_back(conesPC->points[it]);
		}
		cones->width = cones->points.size();
		cones->height = 1;
		cones->is_dense = true;

		// centroid means center point of an object
		Eigen::Vector4f centroid;
		Eigen::Vector4f min;
		Eigen::Vector4f max;
		pcl::compute3DCentroid(*cones, centroid);
		pcl::getMinMax3D(*cones, min, max);

		float bound_x = std::fabs(max[0] - min[0]);
		float bound_y = std::fabs(max[1] - min[1]);
		float bound_z = std::fabs(max[2] - min[2]);

		visualization_msgs::Marker box;
		box.header.frame_id = "livox_frame";
		box.header.stamp = ros::Time();
		box.type = visualization_msgs::Marker::CYLINDER;
		box.action = visualization_msgs::Marker::ADD;
		box.id = id_count;
		box.pose.position.x = centroid[0];
		box.pose.position.y = centroid[1];
		box.pose.position.z = centroid[2];
		box.pose.orientation.x = 0;
		box.pose.orientation.y = 0;
		box.pose.orientation.z = 0;
		box.pose.orientation.w = 1;
		box.scale.x = bound_x;
		box.scale.y = bound_y;
		box.scale.z = bound_z;
		box.color.a = 1.0;
		box.color.r = 1.0;
		box.color.b = 1.0;
		box.color.g = 1.0;
		bounding_boxes.markers.push_back(box);

		// // filter based on the shape of cones
		// if ((0.1 < bound_x) && (bound_x < 0.3) && 
		// 	(0.1 < bound_y) && (bound_y < 0.3) &&
		// 	(0.2 < bound_z) && (bound_z < 0.4) && centroid[2] < 0.3) {
		// 	pcl::PointXYZI tempCentroid;
		// 	tempCentroid.x = centroid[0];
		// 	tempCentroid.y = centroid[1];
		// 	tempCentroid.z = centroid[2];
		// 	clusteredPC->push_back(tempCentroid);
		// }
		pcl::PointXYZI tempCentroid;
		tempCentroid.x = centroid[0];
		tempCentroid.y = centroid[1];
		tempCentroid.z = centroid[2];
		clusteredPC->push_back(tempCentroid);
		id_count++;
	}
	vis_pub.publish(bounding_boxes);
}

void lidar_callback(const sensor_msgs::PointCloud2ConstPtr &msg1, const sensor_msgs::PointCloud2ConstPtr &msg2, const sensor_msgs::PointCloud2ConstPtr &msg3) {
	pcl::PointCloud<point_type>::Ptr rawPC1356(new pcl::PointCloud<point_type>);
	pcl::PointCloud<point_type>::Ptr rawPC1518(new pcl::PointCloud<point_type>);
	pcl::PointCloud<point_type>::Ptr rawPC1712(new pcl::PointCloud<point_type>);
	rawPC1356->clear();
	rawPC1518->clear();
	rawPC1712->clear();

	pcl::fromROSMsg(*msg1, *rawPC1356);
	pcl::fromROSMsg(*msg2, *rawPC1518);
	pcl::fromROSMsg(*msg3, *rawPC1712);

/*  
    ---------------------------------------------------------------------------
                            Region of Interest
    ---------------------------------------------------------------------------
*/  
	pcl::PointCloud<point_type>::Ptr processPC(new pcl::PointCloud<point_type>);
	processPC->clear();
	for (auto &pt : rawPC1356->points) {
		if ((pt.x < 3) || (pt.z > 0.4) || (std::hypot(pt.x, pt.y) > range) || (pt.z < -0.05)) 
			continue;
		processPC->push_back(pt);
	}
	for (auto &pt : rawPC1712->points) {
		if ((pt.x < 3) || (pt.z > 0.4) || (std::hypot(pt.x, pt.y) > range) || (pt.z < -0.05)) 
			continue;
		processPC->push_back(pt);
	}
	for (auto &pt : rawPC1518->points) {
		if ((pt.x < 3) || (pt.z > 0.4) || (std::hypot(pt.x, pt.y) > range) || (pt.z < -0.05)) 
			continue;
		processPC->push_back(pt);
	}

	sensor_msgs::PointCloud2::Ptr ROIPC(new sensor_msgs::PointCloud2);
	pcl::toROSMsg(*processPC, *ROIPC);
	ROIPC->header.stamp = msg1->header.stamp;
	ROIPC->header.frame_id = msg1->header.frame_id;
	ROI_pub.publish(*ROIPC);

/*  
    ---------------------------------------------------------------------------
                            GROUND REMOVAL MODULE
    ---------------------------------------------------------------------------
*/  	
	pcl::PointCloud<pcl::PointXYZI>::Ptr conesPC(new pcl::PointCloud<point_type>);
	std::vector<pcl::PointCloud<point_type>> segBin(nx * ny);
	int index_x, index_y;
	conesPC->clear();

	for (auto &pt : processPC->points) {
		index_x = int(pt.x / dx);
		index_y = int(pt.y / dy) + 100;
		segBin[index_x * ny + index_y].push_back(pt);
	}

	point_type ptH, ptL;
	for (int i = 0; i < nx; i++) {
		for (int j = 0; j < ny; j++) {
			if (segBin[i * ny + j].size() < 1) {
				continue;
			}

			ptH = segBin[i * ny + j].points[0];
			ptL = segBin[i * ny + j].points[0];

			for (auto &pt : segBin[i * ny + j].points) {
				if (ptH.z < pt.z) {
					ptH = pt;
				}
				if (ptL.z > pt.z) {
					ptL = pt;
				}
			}
			
			if ((ptH.z - ptL.z) > (cone_height + cone_height_thre)) {
				segBin[i * ny + j].clear();
			} else if ((ptH.z - ptL.z) < (cone_height - cone_height_thre)) {
				segBin[i * ny + j].clear();
			} else {
				for (auto &pt : segBin[i * ny + j]) {
					if ((pt.z - ptL.z) > 0.05) {
						conesPC->push_back(pt);
					}
				}
			}
		}
	}
	sensor_msgs::PointCloud2::Ptr filterPC(new sensor_msgs::PointCloud2);
	pcl::toROSMsg(*conesPC, *filterPC);
	filterPC->header.stamp = msg1->header.stamp;
	filterPC->header.frame_id = msg1->header.frame_id;
	filter_pub.publish(*filterPC);

	
	if (integral_count < inte_count) {
		inte_flag = false;
		for (auto &pt : conesPC->points) {
			intePC->push_back(pt);
		}
		integral_count++;
	} else {
		inte_flag = true;
	}

	if (conesPC->size() > 0) {
		pcl::PointCloud<point_type>::Ptr clusteredPC(new pcl::PointCloud<point_type>);
		sensor_msgs::PointCloud2::Ptr outPC(new sensor_msgs::PointCloud2);
		clustering(conesPC, clusteredPC, cluster_thre, cluster_min, cluster_max);
		pcl::toROSMsg(*clusteredPC, *outPC);
		outPC->header.stamp = msg1->header.stamp;
		outPC->header.frame_id = msg1->header.frame_id;
		out_pub.publish(*outPC);
	} else {
		ROS_INFO("no points in cone point cloud");
	}
	
	// if (inte_flag) {
	// 	pcl::PointCloud<point_type>::Ptr inte_cluster(new pcl::PointCloud<point_type>);
	// 	sensor_msgs::PointCloud2::Ptr inteOutPC(new sensor_msgs::PointCloud2);
	// 	clustering(intePC, inte_cluster, cluster_thre, cluster_min, cluster_max);
	// 	pcl::toROSMsg(*clusteredPC, *inteOutPC);
	// 	inteOutPC->header.frame_id = carrier.header.frame_id;
	// 	inteOutPC->header.stamp = carrier.header.stamp;
	// 	inte_pub.publish(*inteOutPC);
	// }
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "livox_cluster_node");
	ros::NodeHandle nh("~");

	message_filters::Subscriber<sensor_msgs::PointCloud2> sub1(nh, "/livox/lidar_3WEDH7600113561", 10);
	message_filters::Subscriber<sensor_msgs::PointCloud2> sub2(nh, "/livox/lidar_3WEDH7600117121", 10);
	message_filters::Subscriber<sensor_msgs::PointCloud2> sub3(nh, "/livox/lidar_3WEDH7600115181", 10);

	message_filters::Synchronizer<sync_policy> sync(sync_policy(10), sub1, sub2, sub3);
	sync.registerCallback(boost::bind(&lidar_callback, _1, _2, _3));
	
	out_pub = nh.advertise<sensor_msgs::PointCloud2>("/perception/livox_cluster", 10);
	ROI_pub = nh.advertise<sensor_msgs::PointCloud2>("/perception/lidar_ROI", 10);
	inte_pub = nh.advertise<sensor_msgs::PointCloud2>("/perception/inte_cluster", 10);
	filter_pub = nh.advertise<sensor_msgs::PointCloud2>("/perception/lidar_filtered", 10);
	vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/perception/lidar_bounding_boxes", 10);

	integral_count = 0;
	inte_flag = false;
	ROS_INFO("initialization finished");
	
	ros::spin();
	return 0;
}

