//Recoded by HoGinhang at Thu Nov 4


#include <ros/ros.h>
#include "line_detector.h"
#include <algorithm>

namespace ns_line_detector
{

LineDetector::LineDetector(ros::NodeHandle &nh)
{
	nh_=nh;
    laser_count = 0;
	loadParameters();
	subscribeToTopics();
	publishToTopics();

}

void LineDetector::loadParameters()
{
	if (!nh_.param<std::string>("/planning/lidar_cluster_topic_name",
	                                  lidar_cluster_topic_name_,
	                                  "/perception/lidar_cluster")) {
	ROS_WARN_STREAM("Did not load lidar_cluster_topic_name. Standard value is: " << lidar_cluster_topic_name_);
	}
	if (!nh_.param<std::string>("/planning/end_point_topic_name",
	                                  end_point_topic_name_,
	                                  "/planning/end_point")) {
	ROS_WARN_STREAM("Did not load end_point_topic_name. Standard value is: " << end_point_topic_name_);
	}

	if (!nh_.param<double>("/planning/path_length", path_length, 80)) {
	    ROS_WARN_STREAM("Did not load path_length. Standard value is: " << path_length);
	}
	if (!nh_.param<double>("/planning/allow_angle_error", allow_angle_error, 1.0)) {
	    ROS_WARN_STREAM("Did not load allow_angle_error. Standard value is: " << allow_angle_error);
	}
}


void LineDetector::subscribeToTopics() {
  ROS_INFO("subscribe to topics");
  lidarClusterSubscriber_ = nh_.subscribe(lidar_cluster_topic_name_, 1, &LineDetector::lidarClusterCallback, this);
}

void LineDetector::publishToTopics()
{
	ROS_INFO("publish to topics");
  	endPointPublisher_ = nh_.advertise<geometry_msgs::Point>(end_point_topic_name_, 1);
    PathPublisher_ = nh_.advertise<visualization_msgs::MarkerArray>("planning/visualpath",1,true);
    vis_pub = nh_.advertise<visualization_msgs::MarkerArray>("planning/visualcluster",1,true);
    MiddlePublisher_ = nh_.advertise<visualization_msgs::MarkerArray>("planning/visualmiddle",1,true);
}

void LineDetector::sendMsg()
{
	  endPointPublisher_.publish(end_point_);
}


void LineDetector::lidarClusterCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	pcl::fromROSMsg(*msg,*cluster_ptr_);
    laser_count++;
	//ROS_INFO("lidar size is %ld",cluster_ptr_->points.size());
}

void LineDetector::runAlgorithm() {
    if(!getPath)
        createPath();
    else
        return;
}

//欧式聚类函数，用于稳定点云，反正静态的，对前几帧做聚类都无所谓
void LineDetector::clustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr &conesPC, pcl::PointCloud<pcl::PointXYZI>::Ptr &clusteredPC){
    // Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
	tree->setInputCloud(conesPC);

	std::vector<pcl::PointIndices> clusterIndices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
	ec.setClusterTolerance(1);      //设置近邻搜索的半径
	ec.setMinClusterSize(5);        //设置最小聚类点数
	ec.setMaxClusterSize(100);       //设置最大聚类点数
	ec.setSearchMethod(tree);       //设置搜索方式
	ec.setInputCloud(conesPC);      //设置输入点云
	ec.extract(clusterIndices);     //从点云中提取聚类，并将点云索引保存在cluster_indices中

    visualization_msgs::MarkerArray bounding_boxes;
    int id_count = 0;
    for (const auto &iter : clusterIndices)
    {
        //创建新的点云数据集cones，将所有当前聚类写入到点云数据集中
        pcl::PointCloud<pcl::PointXYZI>::Ptr cones(new pcl::PointCloud<pcl::PointXYZI>);
        for (auto it : iter.indices) {
            cones->points.push_back(conesPC->points[it]);
        }
        cones->width = cones->points.size();
        cones->height = 1;
        cones->is_dense = true;
		// centroid means center point of an object
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*cones, centroid);

        //可视化实现
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
		box.scale.x = 0.5;
		box.scale.y = 0.5;
		box.scale.z = 0.5;
		box.color.a = 1.0;
		box.color.r = 1.0;
		box.color.b = 1.0;
		box.color.g = 1.0;
		bounding_boxes.markers.push_back(box);

        pcl::PointXYZI tempCentroid;
		tempCentroid.x = centroid[0];
		tempCentroid.y = centroid[1];
		tempCentroid.z = centroid[2];
        clusteredPC->push_back(tempCentroid);
        id_count++;
    }
    vis_pub.publish(bounding_boxes);
}


/*
TODO
存在问题是：感知层会有闪烁，会导致单边缺对应点的情况，那么除了起始点，后续点都是不准确的
拟解决方案：在感知不变的情况下，对前10帧聚类，从而得到稳定的双边点，这样可以有多组对应点进行直线拟合————已解决√
目前路径生成方案是：直接利用起始两点做中垂线0.0，好处是不需要多组对应点，比较直接、简单，效果还行
*/
void LineDetector::createPath() {
    //合法性检验
    if(cluster_ptr_->points.size() == 0)
        return;
    //进行10帧聚类，这个10可以考虑设为更大的值，以追求更稳定的点云
    if(laser_count < 10){
        for(const auto &iter : cluster_ptr_->points){
            conesPC->points.push_back(iter);
            conesPC->width = conesPC->points.size();
            conesPC->height = 1;
            conesPC->is_dense = true;
        }
        ROS_INFO("we're now at %d frame, starting to cluster", laser_count);
        return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr clusteredPC(new pcl::PointCloud<pcl::PointXYZI>);
    clusteredPC->points.clear();
    clustering(conesPC, clusteredPC);           //这里获得10帧聚类后的点云clusteredPC

    geometry_msgs::Point StartPoint1_;
    geometry_msgs::Point StartPoint2_;
    double mx,my,dist2base;
    double dist_min = 999;
    bool found_min1_ = false;
    bool found_min2_ = false;

    //搜索第一个最近点，输入至StartPoint1_
    for(int i=0; i<clusteredPC->points.size();i++){
        mx = clusteredPC->points[i].x;
        my = clusteredPC->points[i].y;
        dist2base = hypot(mx,my);
        if(dist2base < dist_min){
            dist_min = dist2base;
            StartPoint1_.x = mx;
            StartPoint1_.y = my;
            found_min1_ = true;
        }
    }

    //搜索第二个最近点，输入至StartPoint2_
    dist_min = 999;
    double dist_to_start1;
    for(int i=0; i<clusteredPC->points.size();i++){
        mx = clusteredPC->points[i].x;
        my = clusteredPC->points[i].y;
        dist_to_start1 = hypot(mx - StartPoint1_.x, my - StartPoint1_.y);
        dist2base = hypot(mx,my);
        if(dist2base < dist_min && dist_to_start1 > 3){
            dist_min = dist2base;
            StartPoint2_.x = mx;
            StartPoint2_.y = my;
            found_min2_ = true;
        }
    }

    //合法性检验
    if(!found_min1_ || !found_min2_){
        ROS_INFO("can not find 2 startpoints");
        return;
    }

	std::vector<geometry_msgs::Point> MiddlePoints;
	std::vector<geometry_msgs::Point> BorderPoints1_;
	std::vector<geometry_msgs::Point> BorderPoints2_;

    //根据两个起始点，搜索两边点,得到两边界点集
    BorderPoints1_.push_back(StartPoint1_);
    BorderPoints2_.push_back(StartPoint2_);
    findNextPoint(StartPoint1_, BorderPoints1_, clusteredPC);
    findNextPoint(StartPoint2_, BorderPoints2_, clusteredPC);

    //根据两边界点集，得出中心点集
    int length_ = std::min(BorderPoints1_.size(), BorderPoints2_.size());
    ROS_INFO("length is %d", length_);
    for(int i=0;i<length_;i++){
        geometry_msgs::Point MiddlePoint;
        MiddlePoint.x = (BorderPoints1_[i].x + BorderPoints2_[i].x)/2;
        MiddlePoint.y = (BorderPoints1_[i].y + BorderPoints2_[i].y)/2;
        MiddlePoints.push_back(MiddlePoint);
    }
    ROS_INFO("we have %d middlepoints", MiddlePoints.size());

    //根据中心点集,调用最小二乘法，拟合出直线函数，然后令x=82，求解出y
    //此处线性拟合只用了两个点，有点浪费线性拟合了；但随着感知距离的增长，两边对应点数的增多，线性拟合的作用将会更大
    std::vector<double> vResult;
    LineFitLeastSquares(MiddlePoints, vResult);
    double px = 82, py;
    py = vResult[0]*px + vResult[1];
    ROS_INFO("Linear Squares we get is %lf", vResult[2]);

    //ATTENTION!!
    //另外一种方法：计算两个StartPoints之间的theta_，他约等于赛道的偏航角，由此可以得到赛道两起始点的相对位姿
    // double theta_;
    // theta_ = atan2(StartPoint2_.x - StartPoint1_.x, StartPoint2_.y - StartPoint1_.y);
    // // ROS_INFO("you can see theta between 2 StartPoints is %lf", theta_);
    // theta_ = -theta_;
    // double px,py;
    // px = 82;
    // py = px * tan(theta_);

    end_point_.x = px;
    end_point_.y = py;
    ROS_INFO("end_point is (x=%lf,y=%lf)",end_point_.x,end_point_.y);
    ROS_INFO("starting sending...");
    getPath = true;
    //把endpoint_沿原点方向做可视化
    visualization_path(MiddlePoints);
}

void LineDetector::findNextPoint(const geometry_msgs::Point &StartPoint, std::vector<geometry_msgs::Point> &BorderPoints, const pcl::PointCloud<pcl::PointXYZI>::Ptr &clusteredPC){
    //向StartPoint前捕捉cone_range个点
    int cone_range = 3;
    double dist_nearest, dist2_Origin, dist_y;
    geometry_msgs::Point OriginalPoint =  StartPoint;

    for(int i=0;i<cone_range;i++){
        dist_nearest = 999;
        int j = 0;
        int index_min = -1;
        for(const auto &iter:clusteredPC->points){
            geometry_msgs::Point matchPoint;
            matchPoint.x = iter.x;
            matchPoint.y = iter.y;
            dist_y = abs(OriginalPoint.y - matchPoint.y);
            dist2_Origin = hypot(matchPoint.x - OriginalPoint.x, matchPoint.y - OriginalPoint.y);
            if(dist2_Origin < dist_nearest && matchPoint.x > OriginalPoint.x && dist_y <1){
                dist_nearest = dist2_Origin;
                index_min = j;
            }
            j++;
        }
        if(index_min >=0){
            OriginalPoint.x = clusteredPC->points[index_min].x;
            OriginalPoint.y = clusteredPC->points[index_min].y;
            BorderPoints.push_back(OriginalPoint);
        }
    }
}

void LineDetector::visualization_path(std::vector<geometry_msgs::Point> MiddlePoints){
    visualization_msgs::Marker Path_;
	visualization_msgs::MarkerArray Path_array;
    visualization_msgs::Marker Middle_;
	visualization_msgs::MarkerArray Middle_array;

    Path_.header.frame_id = "livox_frame";
    Path_.header.stamp = ros::Time::now();
    Path_.ns = "white";
    Path_.action = visualization_msgs::Marker::ADD;
    Path_.type = visualization_msgs::Marker::SPHERE;
    Path_.scale.x = 0.2;
    Path_.scale.y = 0.2;
    Path_.scale.z = 0.2;
    Path_.color.a = 0.2;
    Path_.color.r = 255;
    Path_.color.g = 255;
    Path_.color.b = 255;
    
    for(int i=1;i<=820;i++) {
        Path_.pose.position.x = end_point_.x * i / 820;
        Path_.pose.position.y = end_point_.y * i / 820;
        Path_.id = i;
        Path_array.markers.push_back(Path_);
    }

    Middle_.header.frame_id = "livox_frame";
    Middle_.header.stamp = ros::Time::now();
    Middle_.ns = "yellow";
    Middle_.action = visualization_msgs::Marker::ADD;
    Middle_.type = visualization_msgs::Marker::SPHERE;
    Middle_.scale.x = 0.5;
    Middle_.scale.y = 0.5;
    Middle_.scale.z = 0.5;
    Middle_.color.a = 0.5;
    Middle_.color.r = 255;
    Middle_.color.g = 255;
    Middle_.color.b = 0;
    for(int i=0;i<MiddlePoints.size();i++) {
        Middle_.pose.position.x =MiddlePoints[i].x;
        Middle_.pose.position.y =MiddlePoints[i].y;
        Middle_.id = i;
        Middle_array.markers.push_back(Middle_);
    }

    PathPublisher_.publish(Path_array);
    MiddlePublisher_.publish(Middle_array);
}

/*
最小二乘法拟合直线，y = a*x + b; n组数据; r-相关系数[-1,1],fabs(r)->1,说明x,y之间线性关系好，fabs(r)->0，x,y之间无线性关系，拟合无意义
 a = (n*C - B*D) / (n*A - B*B)
 b = (A*D - B*C) / (n*A - B*B)
 r = E / F
 其中：
 A = sum(Xi * Xi)
 B = sum(Xi)
 C = sum(Xi * Yi)
 D = sum(Yi)
 E = sum((Xi - Xmean)*(Yi - Ymean))
 F = sqrt(sum((Xi - Xmean)*(Xi - Xmean))) * sqrt(sum((Yi - Ymean)*(Yi - Ymean)))
*/
void LineDetector::LineFitLeastSquares(std::vector<geometry_msgs::Point> MiddlePoints, std::vector<double> &vResult)
{
    if(MiddlePoints.size() <= 1){
        return;
    }
	double A = 0.0;
	double B = 0.0;
	double C = 0.0;
	double D = 0.0;
	double E = 0.0;
	double F = 0.0;
    int data_n = MiddlePoints.size();
    
	for (int i=0; i<MiddlePoints.size(); i++)
	{
        double mx,my;
        mx = MiddlePoints[i].x;
        my = MiddlePoints[i].y;
		A += mx*mx;
		B += mx;
		C += mx * my;
		D += my;
	}
 
	// 计算斜率a和截距b
	double a, b, temp = 0;
	if( temp = (data_n*A - B*B) )// 判断分母不为0
	{
		a = (data_n*C - B*D) / temp;
		b = (A*D - B*C) / temp;
	}
	else
	{
		a = 1;
		b = 0;
	}
 
	// 计算相关系数r，求解方差，显示拟合程度
	double Xmean, Ymean;
	Xmean = B / data_n;
	Ymean = D / data_n;
	double tempSumXX = 0.0, tempSumYY = 0.0;
	for (int i=0; i<data_n; i++)
	{
        double mx,my;
        mx = MiddlePoints[i].x;
        my = MiddlePoints[i].y;
		tempSumXX += (mx - Xmean) * (mx - Xmean);
		tempSumYY += (my - Ymean) * (my - Ymean);
		E += (mx - Xmean) * (my - Ymean);
	}
	F = sqrt(tempSumXX) * sqrt(tempSumYY);
	double r;
	r = E / F;
 
	vResult.push_back(a);
	vResult.push_back(b);
	vResult.push_back(r*r);
}


}