/*
 * slam_mapping
 */

/* Author: HoGinhang */

#include "DemoSlamMapping.h"

#include <iostream>
#include <time.h>
#include <memory>

#include "ros/console.h"
#include "std_msgs/String.h"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

//函数重载，不用管
SlamMapping::SlamMapping(){
    init();
}

/*析构函数，自动释放内存，不用管*/
SlamMapping::~SlamMapping(){}

/*SlamMapping的初始化，主要用来读取配置文件中写入的参数*/
void SlamMapping::init()
{
    got_map_ = false;
    got_calibrated_ = false;
    got_clustered_ = false;
    laser_number_ = 0;
    odom_number_=0;
    lapCount_number_=0;
    lapCount_.data = 0;
    map_frame_ = "map";
}

/*开始在线SLAM*/
void SlamMapping::startLiveSlam()
{
    //订阅一些主题，发布一些主题
    MapPublisher_ = node_.advertise<fsd_msgs::Map>("/slam/map", 1, true);
    LocalMapPublisher_ = node_.advertise<fsd_msgs::Map>("/slam/localmap", 1, true);
    VehStatePublisher_ = node_.advertise<fsd_common_msgs::CarState>("/slam/carstate",1,true);
    lapCountPublisher_ = node_.advertise<std_msgs::Int16>("/slam/lapCount",1,true);

    //订阅RTK数据，消息类型未知，等于获得当前位姿
    PathByRTK_ = node_.subscribe("/veh_status/pose", 1, &SlamMapping::odomCallback, this);
    SpeedByRTK_= node_.subscribe("/veh_status/speed", 1, &SlamMapping::SpeedCallback, this);
    AccByRTK_= node_.subscribe("/veh_status/acc", 1, &SlamMapping::AccCallback, this);

    //订阅一次聚类点数据，消息类型未知
    PointDataSubscriber_ = node_.subscribe("/perception/ps_interface", 1, &SlamMapping::laserCallback, this);

    //发布消息
    Msg_to_Pub();

    //打印消息
    std::cout <<"Subscribe PointCloudData & odom!!!"<<std::endl;

}

/*生成local发给控制器，理论上map_本身是按序排列的，只需要根据当前位置，发出前视多少个localmap就ok*/
void SlamMapping::generate_localmap(const Mapping::OrientedPoint &mpose){
    if(lapCount_.data == 0){
        return;
    }
    Mapping::OrientedPoint npose = mpose;
    //第一圈直接返回，然后第二圈进来，肯定是有全局map_了
    //进来先初始化清空一下
    localmap_.cone_high_yellow.clear();
    localmap_.cone_red.clear();
    localmap_.cone_blue.clear();
    localmap_.cone_low_yellow.clear();
    
    //首先将全局地图，全部转换到base_link中，然后基于base_link中的点，找到前视的10个点，将其输出
    //基本原理是，判断点的纵向x是否大于0，然后是否为距离mpose最近的十个点，然后push到localmap_
    Mapping::Point mpoint;
    double dist_min = 999;
    double dist_base;
    int index_min;

    for(int i =0;i<map_.cone_red.size();i++){
        mpoint.x = map_.cone_red[i].x;
        mpoint.y = map_.cone_red[i].y;
        Points_map_to_vehicle(npose, mpoint);

        if(mpoint.x >=0){
            dist_base = hypot(mpoint.x, mpoint.y);
            if(dist_base < dist_min){
                dist_min = dist_base;
                index_min = i;
            }
        }
    }
   
   //找到了在base_link中里车辆纵向、正向最近的点的索引
   //由此，利用索引对应关系，输出map_中red和blue锥桶
    //TODO，注意接口改变！！！

    int cone_range = 10; //表示localmap顺序传递10个点
    geometry_msgs::Point cone_red_local;
    geometry_msgs::Point cone_blue_local;

    for(int i = index_min; i<index_min + cone_range;i++){
        cone_red_local.x =  map_.cone_red[i].x;
        cone_red_local.y =  map_.cone_red[i].y;
        cone_red_local.z =  0;

        cone_blue_local.x =  map_.cone_blue[i].x;
        cone_blue_local.y =  map_.cone_blue[i].y;
        cone_blue_local.z =  0;

        localmap_.cone_red.push_back(cone_red_local);
        localmap_.cone_blue.push_back(cone_blue_local);
    }
}

/*将点从map转换到base_link，输入:要转换的点和当前车辆位姿*/
void SlamMapping::Points_map_to_vehicle(Mapping::OrientedPoint &mpose, Mapping::Point &mpoint){
    Mapping::OrientedPoint Vehicle_pose;
    Vehicle_pose.x = -mpose.x;
    Vehicle_pose.y = -mpose.y;
    Vehicle_pose.theta = -mpose.theta;

    double mx = mpoint.x;
    double my = mpoint.y;
    mpoint.x = Vehicle_pose.x + mx * cos(Vehicle_pose.theta) + my * sin(Vehicle_pose.theta);
    mpoint.y = Vehicle_pose.y - mx * sin(Vehicle_pose.theta) + my *cos(Vehicle_pose.theta);
}

/*将点从base_link转换到map，输入:要转换的点和当前车辆位姿*/
void SlamMapping::Points_vehicle_to_map(Mapping::OrientedPoint &mpose, Mapping::Point &mpoint){
    Mapping::OrientedPoint Vehicle_pose = mpose;
    double mx = mpoint.x;
    double my = mpoint.y;
    mpoint.x = Vehicle_pose.x + mx * cos(Vehicle_pose.theta) + my * sin(Vehicle_pose.theta);
    mpoint.y = Vehicle_pose.y - mx * sin(Vehicle_pose.theta) + my *cos(Vehicle_pose.theta);
}

/*里程计信息的回调函数*/
void SlamMapping::odomCallback(const geometry_msgs::Pose2D::ConstPtr& mpose)
{
    odom_number_ ++;

    //合法性检测
    if(!odom_number_)
        return;

    Mapping::OrientedPoint odom_pose;
    odom_pose.x = mpose->x;
    odom_pose.y = mpose->y;
    odom_pose.theta = mpose->theta;
    VehiclePose.push_back(odom_pose);

    lapCount_Calculate(odom_pose);

    generate_localmap(odom_pose);

    //position at Map
    carstate_.car_state.x = odom_pose.x;
    carstate_.car_state.y = odom_pose.y;
    carstate_.car_state.theta = odom_pose.theta;

    //时间暂时无法同步！！！！！
    //时间序列的索引index，与VehiclePose的索引同步
    ros::Time now_stamp = ros::Time::now();
    TimeSeries.push_back(now_stamp);
}

void SlamMapping::SpeedCallback(const geometry_msgs::Vector3::ConstPtr& mspeed){
    //velocities at Map, 分别是纵向速度、横向速度和横摆角速度
    carstate_.car_state_dt.car_state_dt.x = mspeed->x;
    carstate_.car_state_dt.car_state_dt.y = mspeed->y;
    carstate_.car_state_dt.car_state_dt.theta = mspeed->z;
}

void SlamMapping::AccCallback(const geometry_msgs::Accel::ConstPtr& macc){
    //accleration at Map
    carstate_.car_state_dt.car_state_a.x = macc->linear.x;
    carstate_.car_state_dt.car_state_a.y = macc->linear.y;
    carstate_.car_state_dt.car_state_a.theta = macc->angular.z;
}

/*发布消息*/
void SlamMapping::Msg_to_Pub(){
    VehStatePublisher_.publish(carstate_);
    lapCountPublisher_.publish(lapCount_);
    MapPublisher_.publish(map_);
    LocalMapPublisher_.publish(localmap_);
}

//lapCount用来识别车辆经过第几圈，具体策略是利用定位来确认
//odom是200Hz，Lidar是10Hz
void SlamMapping::lapCount_Calculate(const Mapping::OrientedPoint &odom_pose){
    //lapCount_number帧数小于1000的话，就不进行lapCount计算
    lapCount_number_ ++;
    int lapCount_thershold = 1000;
    if(lapCount_number_ <= lapCount_thershold){
        return;
    }

    Mapping::OrientedPoint base_pose;
    base_pose.x = VehiclePose[0].x;
    base_pose.y = VehiclePose[0].y;
    base_pose.theta = VehiclePose[0].theta;
    double dist_to_basepoint = euclidianDist(base_pose, odom_pose);
    //车辆与原点距离小于0.1m，则认为在原点附近
    double dist_thershold = 0.1;

    if(dist_to_basepoint <= dist_thershold){
        lapCount_.data ++;
        //车辆进过一圈后，lapCount_number_清0
        lapCount_number_ = 0;
    }
    
}

/*激光信息的回调函数，每次节点收到一条消息时都将调用此函数，这个函数也是主要函数*/
void SlamMapping::laserCallback(const sensor_msgs::PointCloud2::ConstPtr& laserCloudMsg)
{
   laser_number_++;
    int i=0;
    //合法性检测
   if(!laser_number_)
    {return;}  

    //0 - BY; 1 - R ; 2 - B ; 3 - SY;
   //将一次聚类点云转化为pcl::PointXYZL
    pcl::PointCloud<pcl::PointXYZL>::Ptr raw_pcl_ptr_ = pcl::PointCloud<pcl::PointXYZL>::Ptr(new pcl::PointCloud<pcl::PointXYZL>);
    pcl::fromROSMsg(*laserCloudMsg, *raw_pcl_ptr_);

    //每帧进来，ScanPoints都要清空
    ScanPoints.label.clear();
    ScanPoints.points.clear();

    //假设一次聚类点的points进来是有次序的，则ScanPoints也是有序的
   for(i=0; i< raw_pcl_ptr_->points.size();i++)
   {
        pcl::PointXYZL pc = raw_pcl_ptr_->points[i];
        Mapping::Point mp;
        mp.x = pc.x;
        mp.y = pc.y;
        ScanPoints.points.push_back(mp);
        ScanPoints.label.push_back(pc.label);
   }

   ros::Time now_stamp = ros::Time::now(); 
   //TODO，start_stamp需要外节点传入
   ros::Time start_stamp;//看发送的start_stamp
       
    mMapPoints.label.clear();
    mMapPoints.points.clear();
    UpdateMap(now_stamp);

    //在CalibrateMap把当前帧扫描到的点，做校正，然后push到realMapPoints里
    if(got_map_)
    {CalibrateMap(laser_number_, start_stamp);}
    
    //聚类时，是对Map上所有点进行聚类
    if(got_calibrated_)
    {
        SecondCluster(laser_number_);
    }

    if(got_clustered_)
    {        
        map_.header.stamp = ros::Time::now();
        map_.header.frame_id =  map_frame_;
        
        //将ClusterPoints的(x,y)放到map_里面去，然后发布出去
        //0 - BY; 1 - R ; 2 - B ; 3 - SY;
        int label_map;
        geometry_msgs::Point point_in_map_;
        for(int j =0;j<ClusterMapPoints.points.size();j++){
            label_map = ClusterMapPoints.label[j];
            point_in_map_.x = ClusterMapPoints.points[j].x;
            point_in_map_.y = ClusterMapPoints.points[j].y;
            point_in_map_.z = 0;

            if(label_map = 0){
                map_.cone_high_yellow.push_back(point_in_map_);
            }
            if(label_map = 1){
                map_.cone_red.push_back(point_in_map_);
            }
            if(label_map = 2){
                map_.cone_blue.push_back(point_in_map_);
            }
            if(label_map = 3){
                map_.cone_low_yellow.push_back(point_in_map_);
            }
            
        }
                 
    }
   
}

/*得到对应时刻t的车辆位姿*/
void SlamMapping::getVehiclePose(Mapping::OrientedPoint& Vehicle_pose, const ros::Time& t){
    int index_to_time = 0;
    for(int i=0;i<TimeSeries.size();i++){
        if(t ==TimeSeries[i])
        index_to_time = i;
    }
    Vehicle_pose = VehiclePose[index_to_time];
    //如果得不到index_to_time的话，要返回false，但要避免index_to_time==0的情况
}

/*地图更新,*/
//UpdateMap，将ScanPoints扫描到的点，转为Map上的点，然后pushback到mMapPoints上
void SlamMapping::UpdateMap(const ros::Time& t)
{
    ROS_DEBUG("Update map");
    //做已知定位的负责建图，Vehicle_pose表示当前车辆位姿
    Mapping::OrientedPoint Vehicle_pose;
    getVehiclePose(Vehicle_pose, t);

    //x轴纵向，y轴横向，z轴朝下
    double mx , my;
    Mapping::Point Map_Points;
    for (int i=0;i<ScanPoints.points.size();i++){
        mx = ScanPoints.points[i].x;
        my = ScanPoints.points[i].y;
        Map_Points.x = Vehicle_pose.x + mx * cos(Vehicle_pose.theta) + my * sin(Vehicle_pose.theta);
        Map_Points.y = Vehicle_pose.y - mx * sin(Vehicle_pose.theta) + my *cos(Vehicle_pose.theta);
        mMapPoints.points.push_back(Map_Points);
        mMapPoints.label.push_back(ScanPoints.label[i]);
    }

    got_map_ = true;
}

/*地图校正*/
void SlamMapping::CalibrateMap(const int laser_number_, const ros::Time& t)
{
    ros::Time start_stamp =  t;
    ros::Time end_stamp = ros::Time::now();
    Mapping::OrientedPoint start_pose;
    Mapping::OrientedPoint end_pose;

    getVehiclePose(start_pose,start_stamp);
    getVehiclePose(end_pose,end_stamp);

    Mapping::OrientedPoint start_to_end = end_pose - start_pose;
    Mapping::Point Map_Points;

    double mx,my;
    for(int i=0;i<mMapPoints.points.size();i++){
        mx = mMapPoints.points[i].x;
        my = mMapPoints.points[i].y;
        Map_Points.x = start_to_end.x + mx * cos(start_to_end.theta)-my * sin(start_to_end.theta);
        Map_Points.y = start_to_end.y + mx * sin(start_to_end.theta)+my *cos(start_to_end.theta);
        CalibratedMapPoints.points.push_back(Map_Points);
        CalibratedMapPoints.label.push_back(mMapPoints.label[i]);

        //遍历CalibratedMapPoints，如果当前Map_Points与Cali..Points里的点距离都大于1.5m，那就算是一个新的簇
        float thershold_dist = 1.5;
        
        //激光第一帧的一次聚类点全算中心点
        if(laser_number_ == 1){
            CenteredPoints.push_back(Map_Points);
            CenteredLabel.push_back(mMapPoints.label[i]);
        }
        else{
            for(i = 0;i<CenteredPoints.size();i++){
                float dist = euclidianDist(Map_Points, CenteredPoints[i]);
                if(dist < thershold_dist){
                    break;
                }
                CenteredPoints.push_back(Map_Points);
                CenteredLabel.push_back(mMapPoints.label[i]);
            }
        }
    }

    got_calibrated_ = true;

}

/*二次聚类，输入为CalibratedMapPoints和当前簇数*/
void SlamMapping::SecondCluster(const int laser_number_){
    //每5组数据后才开始聚一次类  
    /*
    int thershold_number  = 5;
    if(laser_number_ % thershold_number != 0)
    {
        return;
    }
    */

    //按照当前的传入数据点，判断一共有多少簇，初始化K个中心点
    //每一帧K都会变，噢所以每一帧进来，都要重新把CalibratedMapPoints上所有的点都聚类一次
    //每一帧，SortOutPoints都要清空一次，重新定义一遍
    K = CenteredPoints.size();
    std::vector<std::vector<Mapping::Point>> SortOutPoints(K);
    for(int i=0;i<K;i++){
        SortOutPoints[i].clear();
    }

    //label从0到K，代表K个簇
    int label_cluster = 0;
    Mapping::Point SortPoints;

    //遍历CalibratedMapPoints上的点，进行归类，获得label_cluster，根据label_cluster，装进SortOutPoints中
    for(int i =0; i < CalibratedMapPoints.points.size(); i++){
        label_cluster = clusterOfMap(CalibratedMapPoints.points[i]);
        SortPoints.x = CalibratedMapPoints.points[i].x;
        SortPoints.y = CalibratedMapPoints.points[i].y;
        //怎么把分好类的label的CalibratedMapPoints，push到SortOutPoints里
        SortOutPoints[label_cluster].push_back(SortPoints);
    }

    //根据SortOutPoints中每一个簇，归纳出一个中心点
    getClusterPoints(SortOutPoints);

    //到这里，ClusterMapPoints肯定是聚类完了，而且应该是各点的均值，含有label值
    got_clustered_ = true;
}

//根据质心，决定当前点属于哪个簇
int SlamMapping::clusterOfMap(Mapping::Point Map_Point) {
	float dist = euclidianDist(CenteredPoints[0], Map_Point);
	float tmp;
	int label = 0;
	for (int i = 1; i < CenteredPoints.size(); i++) {
		tmp = euclidianDist(CenteredPoints[i], Map_Point);
		if (tmp<dist) { dist = tmp; label = i; }
	}
	return label;
}

//取出各簇的点的平均值
void SlamMapping::getClusterPoints(std::vector<std::vector<Mapping::Point>> SortOutPoints){
    double x_sum=0;
    double y_sum=0;
    double x_ave;
    double y_ave;
    Mapping::Point AveragePoint;

    for(int i=0;i<K;i++ ){
        for(int j=0;j<SortOutPoints[i].size();j++){
            x_sum += SortOutPoints[i][j].x;
            y_sum += SortOutPoints[i][j].y;
        }
        x_ave = x_sum / SortOutPoints[i].size();
        y_ave = y_sum / SortOutPoints[i].size();
        AveragePoint.x = x_ave;
        AveragePoint.y = y_ave;
        ClusterMapPoints.points.push_back(AveragePoint);
        ClusterMapPoints.label.push_back(CenteredLabel[i]);
    }
        
}
