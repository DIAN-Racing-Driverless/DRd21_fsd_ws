/*
 * slam_mapping
 */

/* Author: HoGinhang */

#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Accel.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Float64.h"
#include "fsd_msgs/Map.h"
#include "fsd_common_msgs/CarState.h"
#include "fsd_common_msgs/CarStateDt.h"
#include "std_msgs/Int16.h"

#include"Point.hpp"

struct MapPoints
{
    //points和label的索引是对应的
    std::vector<Mapping::Point> points;
    std::vector<int> label;
};

class SlamMapping
{
    private:
        ros::NodeHandle node_;
        ros::Publisher MapPublisher_; 
        ros::Publisher LocalMapPublisher_;
        ros::Publisher VehStatePublisher_;

        ros::ServiceServer MapServer_;     
        ros::Publisher lapCountPublisher_;   //发布循迹圈数
        
        ros::Subscriber PointDataSubscriber_;   //订阅一次聚类点的数据，消息类型是sensor_msgs::PointCloud

        //rawdata接受的是sensor_msgs::PointCloud2，要转化为pcl::PointCloud<pcl::PointXYZI>
        //随后在转化为自定义的二维的Mapping::Point
        //等于说在接受到消息的时候，在回调函数里要进行点云的数据类型转换
        
        /*用于直接订阅三个激光雷达数据
        ros::Subscriber rawDataSubscriber1_;    
        ros::Subscriber rawDataSubscriber2_;
        ros::Subscriber rawDataSubscriber3_;
        */
       std::string map_frame_;

        ros::Subscriber PathByRTK_;                 //用于订阅RTK的Path，系列的点
        ros::Subscriber SpeedByRTK_;
        ros::Subscriber AccByRTK_;

        ros::NodeHandle private_node_;      //用于处理自身参数初始化

        bool got_map_;
        bool got_calibrated_;
        bool got_clustered_;
        
        int laser_number_;    //激光数，表示激光数据在第几帧
        int odom_number_;    //odom数，表示odom数据在第几帧
        int lapCount_number_;    //lap计算帧数，表示每圈进来在第几帧

        void UpdateMap(const ros::Time& t);
        void getVehiclePose(Mapping::OrientedPoint& Vehicle_pose, const ros::Time& t);
        void CalibrateMap(const int laser_number_, const ros::Time& t);

        void SecondCluster(const int laser_number_);
        int clusterOfMap(Mapping::Point Map_Point);
        void getClusterPoints(std::vector<std::vector<Mapping::Point>> SortOutPoints);

        void lapCount_Calculate(const Mapping::OrientedPoint &odom_pose);
        void generate_localmap(const Mapping::OrientedPoint &mpose);

        void Points_map_to_vehicle(Mapping::OrientedPoint &mpose, Mapping::Point &mpoint);
        void Points_vehicle_to_map(Mapping::OrientedPoint &mpose, Mapping::Point &mpoint);

    public:
        SlamMapping();
        SlamMapping(ros::NodeHandle& nh, ros::NodeHandle& pnh);
        ~SlamMapping(); /*析构函数*/

        void init(); 
        void startLiveSlam(); 
        //对bag进行建图，暂时放弃
        //void startReplay(const std::string & bag_fname, std::string scan_topic);
        
        void odomCallback(const geometry_msgs::Pose2D::ConstPtr& mpose);
        void SpeedCallback(const geometry_msgs::Vector3::ConstPtr& mspeed);
        void AccCallback(const geometry_msgs::Accel::ConstPtr& macc);

        void laserCallback(const sensor_msgs::PointCloud2::ConstPtr& laserCloudMsg);
        void Msg_to_Pub();


        //K表示簇数，是一个动态的变量
        int K;

        //车辆位姿
        std::vector<Mapping::OrientedPoint> VehiclePose;
        std::vector<ros::Time> TimeSeries;

        //ScanMapPoints表示当前帧扫描到的所有点的信息，是位于base_link上的
        struct MapPoints ScanPoints;
        //mMapPoints表示当前帧扫描到的所有点的信息，是位于map上的
        struct MapPoints mMapPoints;
        //CalibratedMapPoints表示校正点的信息，位于map上
        struct MapPoints CalibratedMapPoints;

        //CenteredPoints是Point的容器，CenteredPoints与CenteredLabel索引对应
        std::vector<Mapping::Point> CenteredPoints;
        std::vector<int> CenteredLabel;

        //ClusterMapPoints表示地图上的所有点的信息，是位于map上的，是最后要输出的点
        struct MapPoints ClusterMapPoints;
              
       //与Node通信有关
        fsd_msgs::Map map_;
        fsd_msgs::Map localmap_;
        fsd_common_msgs::CarState carstate_;
        std_msgs::Int16 lapCount_;
};

