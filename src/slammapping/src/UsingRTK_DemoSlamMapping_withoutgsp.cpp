/*
 * slam_mapping
 * Now we use RTK to SLAM before 10.26
 * but now we use IMU to SLAM at 11.07
 * Modified by HoGinhang at Sun Nov 7
 */

/* Author: HoGinhang */

#include "Fortest_DemoSlamMapping.h"

#include <iostream>
#include <time.h>
#include <memory>

#include "ros/console.h"
#include "std_msgs/String.h"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include "fsd_common_msgs/Map.h"
#include "fsd_common_msgs/Cone.h"

#define M_PI 3.1415926535897932384626433832795

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
    laser_number_now = 0;
    map_frame_ = "map";
}

/*开始在线SLAM*/
void SlamMapping::startLiveSlam()
{
    //订阅一些主题，发布一些主题
    MapPublisher_ = node_.advertise<fsd_msgs::Map>("/slam/map", 1, true);
    LocalMapPublisher_ = node_.advertise<fsd_msgs::Map>("/slam/localmap", 1, true);
    VehStatePublisher_ = node_.advertise<fsd_common_msgs::CarState>("/slam/carstate",1,true);
    lapCountPublisher_ = node_.advertise<std_msgs::Int16>("/slam/lapcount",1,true);
    
    VisualPathPublisher_ = node_.advertise<nav_msgs::Path>("/slam/visualpath",1,true);
    Red_VisualMapPublisher_ = node_.advertise<visualization_msgs::MarkerArray>("/slam/visualmapred",1,true);
    Blue_VisualMapPublisher_ = node_.advertise<visualization_msgs::MarkerArray>("/slam/visualmapblue",1,true);
    LocalMap_VisualMapPublisher_ = node_.advertise<visualization_msgs::MarkerArray>("/slam/visualocalmap",1,true);
    NeighborMap_VisualMapPublisher_= node_.advertise<visualization_msgs::MarkerArray>("/slam/visuaneighbormap",1,true);

    //订阅RTK数据，消息类型未知，等于获得当前位姿
    PathByRTK_ = node_.subscribe("/veh_status/pose", 1, &SlamMapping::odomCallback, this);
    SpeedByRTK_= node_.subscribe("/veh_status/speed", 1, &SlamMapping::SpeedCallback, this);
    AccByRTK_= node_.subscribe("/veh_status/acc", 1, &SlamMapping::AccCallback, this);

    //订阅一次聚类点数据，消息类型未知
    PointDataSubscriber_ = node_.subscribe("/perception/ps_interface", 1, &SlamMapping::laserCallback, this);

    //打印消息
    std::cout <<"Subscribe PointCloudData & odom!!!"<<std::endl;

}

void SlamMapping::run_alogrithm(){

    now_stamp = ros::Time::now();
    lapCount_Calculate(odom_pose);
    generate_localmap(odom_pose);
    if(lapCount_.data ==1){
        laser_number_now = 0;
    }
    if(lapCount_.data!=0 && laser_number_now >= 50){
        ROS_INFO("Mapping doesn't work now");
        return;
    }
    map_.cone_red.clear();
    map_.cone_blue.clear();
    map_.cone_low_yellow.clear();
    map_.cone_high_yellow.clear();

    UpdateCones_red.clear();
    UpdateCones_blue.clear();
    // UpdateCones_yellow.clear();
    // UpdateCones_unknow.clear();

    //进行已知位姿的建图
    UpdateMap(now_stamp, ScanCones_red, UpdateCones_red, HistoryUpdateCone_red);
    UpdateMap(now_stamp, ScanCones_blue, UpdateCones_blue, HistoryUpdateCone_blue);
    // UpdateMap(now_stamp, ScanCones_yellow, UpdateCones_yellow, HistoryUpdateCone_yellow);
    // UpdateMap(now_stamp, ScanCones_unknow, UpdateCones_unknow, HistoryUpdateCone_unknow);

    //增加聚类簇
    generate_CenterCone(CenterCone_red, UpdateCones_red);
    generate_CenterCone(CenterCone_blue, UpdateCones_blue);
    // generate_CenterCone(CenterCone_yellow, UpdateCones_yellow);
    // generate_CenterCone(CenterCone_unknow, UpdateCones_unknow);
    
    //聚类时，是对Map上所有点进行聚类，Map上的UpdateCones根据CenterCone进行聚类，最终得到ClusterCone
    SecondCluster(ClusterCone_red, CenterCone_red, UpdateCones_red,HistoryUpdateCone_red);
    SecondCluster(ClusterCone_blue, CenterCone_blue, UpdateCones_blue, HistoryUpdateCone_blue);
    // SecondCluster(ClusterCone_yellow, CenterCone_yellow, UpdateCones_yellow, HistoryUpdateCone_yellow);
    // SecondCluster(ClusterCone_unknow, CenterCone_unknow, UpdateCones_unknow, HistoryUpdateCone_unknow);
    
    //将二次聚类点转到fsd_msgs::Map上
    Switch_to_Map(ClusterCone_red,ClusterCone_blue, ClusterCone_yellow, ClusterCone_unknow);

}

/*生成local发给控制器，理论上map_本身是按序排列的，只需要根据当前位置，发出前视多少个localmap就ok*/
void SlamMapping::generate_localmap(const Mapping::OrientedPoint &mpose){
    if(lapCount_.data == 0){
        ROS_INFO("generate_localmap default Successful");
        return;
    }
    localmap_lastframe = localmap_;
    neighbormap_lastframe = neighbormap_;
    Mapping::OrientedPoint npose;
    npose.x = mpose.x;
    npose.y = mpose.y;
    npose.theta = mpose.theta;

    //第一圈直接返回，然后第二圈进来，肯定是有全局map_了
    localmap_.cone_high_yellow.clear();
    localmap_.cone_red.clear();
    localmap_.cone_blue.clear();
    localmap_.cone_low_yellow.clear();

    neighbormap_.cone_high_yellow.clear();
    neighbormap_.cone_blue.clear();
    neighbormap_.cone_red.clear();
    neighbormap_.cone_low_yellow.clear();
    
    produce_localmap(localmap_.cone_blue, map_.cone_blue,npose, neighbormap_.cone_blue);
    produce_localmap(localmap_.cone_red, map_.cone_red,npose, neighbormap_.cone_red);
    
    if(localmap_.cone_blue.size()==0 || localmap_.cone_red.size()==0){
        localmap_ = localmap_lastframe;
        ROS_INFO("FUCK YOU!!!");
    }
    if(neighbormap_.cone_blue.size()==0 || neighbormap_.cone_red.size()==0){
        neighbormap_ = neighbormap_lastframe;
    }
    ROS_INFO("generate_localmap Successful");

}

void SlamMapping::produce_localmap(std::vector<geometry_msgs::Point> &mlocalcones, std::vector<geometry_msgs::Point> &mapcones, Mapping::OrientedPoint &npose, std::vector<geometry_msgs::Point> &mneighborcones){
    //首先将全局地图，全部转换到base_link中，然后基于base_link中的点，找到前视的10个点，将其输出
    //基本原理是，判断点的纵向x是否大于0，然后是否为距离mpose最近的十个点，然后push到localmap_
    geometry_msgs::Point mpoint;
    geometry_msgs::Point point_base_link;
    std::vector<geometry_msgs::Point> basemap_;
    std::vector<geometry_msgs::Point> trans_map;

    basemap_.clear();
    trans_map.clear();
    double dist_min_plus = 999;
    double dist_min_minor = 999;
    double dist_base_plus;
    double dist_base_minor;
    //plus正向最近点、minor负向最近点
    int index_min_plus;
    int index_min_minor;
    bool index_min_plus_notready = true;
    bool index_min_minor_notready = true;
    bool flag_relate_base = false;

    // for(int i =0;i<mapcones.size();i++)

    int index_mapcone=0;
    for(const auto &iter: mapcones){
        mpoint.x = iter.x;
        mpoint.y = iter.y;
        Points_map_to_vehicle(npose, mpoint);
        point_base_link.x = mpoint.x;
        point_base_link.y = mpoint.y;
        //将当前map坐标系下的点全转到base_link下,索引与map_相同
        basemap_.push_back(point_base_link);
        // trans_map.push_back(point_base_link);

        //找当前车辆位置的对应点(正向)的索引
        if(mpoint.x >=0){
            dist_base_plus = hypot(mpoint.x, mpoint.y);
            if(dist_base_plus < dist_min_plus){
                dist_min_plus = dist_base_plus;
                index_min_plus = index_mapcone;
                index_min_plus_notready = false;
            }
        }

        //找当前车辆位置对应点(负向)的索引
        if(mpoint.x <0){
            dist_base_minor = hypot(mpoint.x, mpoint.y);
            if(dist_base_minor < dist_min_minor){
                dist_min_minor = dist_base_minor;
                index_min_minor = index_mapcone;
                index_min_minor_notready = false;
            }
        }
        index_mapcone++;
    }

    if(index_min_plus_notready || index_min_minor_notready){
        ROS_INFO("index_min_notready is true");
        return;
    }

    double x_;
    double y_;
    fsd::Vec_f wx, wy;
    //利用斜率计算前进插点，y-y0 = k * (x - x0)
    if(basemap_[index_min_minor].x - basemap_[index_min_plus].x != 0 && basemap_[index_min_plus].x != 0){
        double rate_ = (basemap_[index_min_minor].y - basemap_[index_min_plus].y) / (basemap_[index_min_minor].x - basemap_[index_min_plus].x);
        x_ = 0;
        y_ = rate_ * (-basemap_[index_min_plus].x) + basemap_[index_min_plus].y;
        wx.push_back(0);
        wy.push_back(y_);
        geometry_msgs::Point mfirstpoint;
        mfirstpoint.x = x_;
        mfirstpoint.y = y_;
        Points_vehicle_to_map(npose, mfirstpoint);
        mneighborcones.push_back(mfirstpoint);
    }

    //再把正向最小点放进去,先检验正向最小点与前插点是否重合
    mneighborcones.push_back(mapcones[index_min_plus]);
    wx.push_back(basemap_[index_min_plus].x);
    wy.push_back(basemap_[index_min_plus].y);

    // ROS_INFO("after related Successful");
    //找到相关点，循环取当前相关点对应的邻近点
    int cone_range = 10; //表示localmap顺序传递5个点
    double x_next, y_next;
    double x_relate, y_relate;
    double dist_thershold = 6;
    int relate_point=0;
    std::vector<geometry_msgs::Point> dist_for_neighbor;
    std::vector<int> index_dist_for_neighbor;

    Mapping::OrientedPoint trans_pose;
    double dist_last;
    int index_last_min;
    int index_now = index_min_minor;
    int index_next = index_min_plus;   
    
    
    for(int j=0;j<cone_range;j++){
        bool HavenotFoundNeighbor = true;
        double dist_last_min = 999;

        calculate_base_pose(basemap_[index_now],basemap_[index_next],trans_pose);

        trans_map = basemap_;
        int index_transcone = 0;
        for(const auto &iter:trans_map){
            geometry_msgs::Point trans_point;
            trans_point.x = iter.x;
            trans_point.y = iter.y;
            Points_map_to_vehicle(trans_pose, trans_point);
            dist_last = hypot(trans_point.x,trans_point.y);
            if(dist_last <= dist_thershold && trans_point.x >0){
                if(dist_last < dist_last_min){
                    dist_last_min = dist_last;
                    index_last_min = index_transcone;
                    HavenotFoundNeighbor = false;
                }
            }   
            index_transcone++;         
        }

        index_now = index_next;
        index_next = index_last_min;
        if(!HavenotFoundNeighbor){
            relate_point++;
            wx.push_back(basemap_[index_last_min].x);
            wy.push_back(basemap_[index_last_min].y);
            mneighborcones.push_back(mapcones[index_last_min]);
        }
        
    }

    // for(int j=0;j<cone_range;j++){
    //     bool HavenotFoundNeighbor = true;
    //     bool isExtremum = true;
    //     dist_for_neighbor.clear();
    //     index_dist_for_neighbor.clear();
    //     //找距离门槛内的点
    //     for(int i =0;i<basemap_.size();i++){
    //         x_next = basemap_[i].x;
    //         y_next = basemap_[i].y;
    //         x_relate = basemap_[index_min_plus].x;
    //         y_relate = basemap_[index_min_plus].y;
    //         if(hypot(x_next-x_relate,y_next-y_relate) <= dist_thershold){
    //             dist_for_neighbor.push_back(basemap_[i]);
    //             index_dist_for_neighbor.push_back(i);
    //         }
    //     }
    //     //判断是否为区间极点
    //     for(int i=0;i<dist_for_neighbor.size();i++){
    //         x_next = dist_for_neighbor[i].x;
    //         x_relate = basemap_[index_min_plus].x;
    //         if(x_next > x_relate){
    //             isExtremum = false;
    //         }
    //     }
    //     double mdist_;
    //     double dist_min=999;
    //     int index_neighbor;
    //     //若不是区间极值，则返回最近的x大于零的点,若是,则返回最近的|y|大于零的点
    //     for(int i=0;i<dist_for_neighbor.size();i++){
    //         if(isExtremum){
    //             if(abs(dist_for_neighbor[i].y) > abs(y_relate)){
    //                 mdist_ = hypot(dist_for_neighbor[i].x - x_relate, dist_for_neighbor[i].y - y_relate);
    //                 if(mdist_ < dist_min){
    //                     index_neighbor = index_dist_for_neighbor[i];
    //                     dist_min = mdist_;
    //                     HavenotFoundNeighbor = false;
    //                     relate_point++;
    //                 }
    //             }
    //         }
    //         else{
    //             if(dist_for_neighbor[i].x > x_relate){
    //                 mdist_ = hypot(dist_for_neighbor[i].x - x_relate, dist_for_neighbor[i].y - y_relate);
    //                 if(mdist_ < dist_min){
    //                     index_neighbor = index_dist_for_neighbor[i];
    //                     dist_min = mdist_;
    //                     HavenotFoundNeighbor = false;
    //                     relate_point++;
    //                 }
    //             }        
    //         }
    //     }
    //     if(!HavenotFoundNeighbor){
    //         index_min_plus = index_neighbor;
    //         Mapping::Point neighbor_pt;
    //         geometry_msgs::Point neighbor_point;
    //         neighbor_pt.x = basemap_[index_neighbor].x;
    //         neighbor_pt.y = basemap_[index_neighbor].y;
    //         wx.push_back(basemap_[index_neighbor].x);
    //         wy.push_back(basemap_[index_neighbor].y);
    //         Points_vehicle_to_map(npose, neighbor_pt);
    //         neighbor_point.x = neighbor_pt.x;
    //         neighbor_point.y = neighbor_pt.y;
    //         mneighborcones.push_back(neighbor_point);       
    //     }
    // }

    ROS_INFO("after found neighbor Successful, have found %d points", relate_point);
    fsd::Spline2D spline(wx, wy);
    double interval = 0.5;
    double line_x, line_y;
    int splinepointcount = 0;
    for (float i = 0; i < spline.s.back(); i += interval) {
        std::array<float, 2> point_ = spline.calc_postion(i);
        line_x = point_[0];
        line_y = point_[1];
        geometry_msgs::Point Interpolating_point;
        Interpolating_point.x = line_x;
        Interpolating_point.y = line_y;
        Points_vehicle_to_map(npose, Interpolating_point);
        mlocalcones.push_back(Interpolating_point);
        splinepointcount++;
        if(splinepointcount>70){
            break;
        }
    }
    ROS_INFO("spline have  %d points", splinepointcount);
}

void SlamMapping::calculate_base_pose(const geometry_msgs::Point &point_now, const geometry_msgs::Point &point_next, Mapping::OrientedPoint &mpose){
    double x_now = point_now.x;
    double y_now = point_now.y;
    double x_next = point_next.x;
    double y_next = point_next.y;
    double x_pose, y_pose, theta_pose;

    if(x_next == x_now){
        x_pose = x_next;
        y_pose = y_next;
        theta_pose = M_PI/2;
    }
    else if(x_next < x_now){
        x_pose = x_next;
        y_pose = y_next;
        double rate = (y_next - y_now)/(x_next - x_now);
        theta_pose =atan(rate);
        if(theta_pose > 0){
            theta_pose = theta_pose - M_PI;
        }
        else{
            theta_pose = theta_pose + M_PI;
        }
    }
    else{
        x_pose = x_next;
        y_pose = y_next;
        double rate = (y_next - y_now)/(x_next - x_now);
        theta_pose = atan(rate);
    }

    mpose.x = x_pose;
    mpose.y = y_pose;
    mpose.theta = theta_pose;

}

/*将二次聚类点云转换到fsd_msgs::Map中*/
void SlamMapping::Switch_to_Map(const std::vector<fsd_common_msgs::Cone> &ClusterCone_red, const std::vector<fsd_common_msgs::Cone> &ClusterCone_blue, const std::vector<fsd_common_msgs::Cone> &ClusterCone_yellow, const std::vector<fsd_common_msgs::Cone> &ClusterCone_unknow){
    map_.header.stamp = ros::Time::now();
    map_.header.frame_id =  map_frame_;
    
    //将ClusterCone的(x,y)放到map_里面去，然后发布出去
    geometry_msgs::Point point_in_map_;
    // for(int i =0;i<ClusterCone_red.size();i++)
    
    for(int i =0;i<ClusterCone_red.size();i++){
        point_in_map_.x = ClusterCone_red[i].position.x;
        point_in_map_.y = ClusterCone_red[i].position.y;
        point_in_map_.z = 0;
        map_.cone_red.push_back(point_in_map_);
    }
    for(int i =0;i<ClusterCone_blue.size();i++){
        point_in_map_.x = ClusterCone_blue[i].position.x;
        point_in_map_.y = ClusterCone_blue[i].position.y;
        point_in_map_.z = 0;
        map_.cone_blue.push_back(point_in_map_);
    }
    // for(int i =0;i<ClusterCone_yellow.size();i++){
    //     point_in_map_.x = ClusterCone_yellow[i].position.x;
    //     point_in_map_.y = ClusterCone_yellow[i].position.y;
    //     point_in_map_.z = 0;
    //     map_.cone_low_yellow.push_back(point_in_map_);
    // }
    // //暂时认为unkonw与high_yellow对应一致
    // for(int i =0;i<ClusterCone_unknow.size();i++){
    //     point_in_map_.x = ClusterCone_unknow[i].position.x;
    //     point_in_map_.y = ClusterCone_unknow[i].position.y;
    //     point_in_map_.z = 0;
    //     map_.cone_high_yellow.push_back(point_in_map_);
    // }

    ROS_INFO("Quantity of Red Cone is %d", map_.cone_red.size());
    ROS_INFO("Quantity of Blue Cone is %d", map_.cone_blue.size());
        
}

/*将点从map转换到base_link，输入:要转换的点和当前车辆位姿*/
void SlamMapping::Points_map_to_vehicle(Mapping::OrientedPoint &mpose, geometry_msgs::Point &mpoint){
    double vtheta = mpose.theta;
    double vx = mpose.x;
    double vy = mpose.y;
    double mx = mpoint.x;
    double my = mpoint.y;
    
    double deltax =mx - vx;
    double deltay = my - vy;

    mpoint.x = deltax * cos(vtheta) + deltay * sin(vtheta);
    mpoint.y = deltay * cos(vtheta) - deltax * sin(vtheta);
}

/*将点从base_link转换到map，输入:要转换的点和当前车辆位姿*/
void SlamMapping::Points_vehicle_to_map(Mapping::OrientedPoint &mpose, geometry_msgs::Point &mpoint){
    Mapping::OrientedPoint Vehicle_pose = mpose;
    double mx = mpoint.x;
    double my = mpoint.y;
    mpoint.x = Vehicle_pose.x + mx * cos(Vehicle_pose.theta) - my * sin(Vehicle_pose.theta);
    mpoint.y = Vehicle_pose.y + mx * sin(Vehicle_pose.theta) + my *cos(Vehicle_pose.theta);
}

/*里程计信息的回调函数*/
void SlamMapping::odomCallback(const geometry_msgs::Pose2D::ConstPtr& mpose)
{
    odom_number_ ++;
    ROS_INFO("We are now at %d frame", odom_number_);
    odom_pose.x = mpose->x;
    odom_pose.y = mpose->y;
    odom_pose.theta = mpose->theta;
    // VehiclePose.push_back(odom_pose);

    if(odom_number_ <= 20){
        StartPose.x = odom_pose.x + 8;
        StartPose.y = odom_pose.y + 0;
        // StartPose.x = odom_pose.x+5;
        // StartPose.y = odom_pose.y;
        StartPose.theta = odom_pose.theta;
    }
    ROS_INFO("StartPose is (%lf, %lf)", StartPose.x, StartPose.y);
    // ROS_INFO("No Wrong in generate_localmap");
    // lapCount_Calculate(odom_pose);

    //position at Map
    carstate_.car_state.x = odom_pose.x;
    carstate_.car_state.y = odom_pose.y;
    carstate_.car_state.theta = odom_pose.theta;
    // ROS_INFO("OdomCallback Successful");
}

void SlamMapping::SpeedCallback(const geometry_msgs::Vector3::ConstPtr& mspeed){
    //velocities at Map, 分别是纵向速度、横向速度和横摆角速度
    carstate_.car_state_dt.car_state_dt.x = mspeed->x;
    carstate_.car_state_dt.car_state_dt.y = mspeed->y;
    carstate_.car_state_dt.car_state_dt.theta = mspeed->z;
    // ROS_INFO("SpeedCallback Successful");
}

void SlamMapping::AccCallback(const geometry_msgs::Accel::ConstPtr& macc){
    //accleration at Map
    carstate_.car_state_dt.car_state_a.x = macc->linear.x;
    carstate_.car_state_dt.car_state_a.y = macc->linear.y;
    carstate_.car_state_dt.car_state_a.theta = macc->angular.z;
    // ROS_INFO("AccCallback Successful");
}

/*发布消息*/
void SlamMapping::Msg_to_Pub(){
    VehStatePublisher_.publish(carstate_);
    lapCountPublisher_.publish(lapCount_);
    MapPublisher_.publish(map_);
    // LocalMapPublisher_.publish(localmap_);
    LocalMapPublisher_.publish(neighbormap_);
    VisualPathPublisher_.publish(mpath_);
    Red_VisualMapPublisher_.publish(Map_RedConeArray);
    Blue_VisualMapPublisher_.publish(Map_BlueConeArray);
    LocalMap_VisualMapPublisher_.publish(LocalMap_ConeArray);
    NeighborMap_VisualMapPublisher_.publish(Neighbor_ConeArray);
    
    // ROS_INFO("Msg_to_Pub Successful");
}

void SlamMapping::Visualization_Path(){
    geometry_msgs::PoseStamped mposestamp_;

    mpath_.header.stamp = ros::Time::now();
    mpath_.header.frame_id = "map";

    mposestamp_.header.stamp = ros::Time::now();
    mposestamp_.pose.position.x = carstate_.car_state.x;
    mposestamp_.pose.position.y = carstate_.car_state.y;
    mposestamp_.pose.position.z = 0;

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(carstate_.car_state.theta);
    mposestamp_.pose.orientation.x = goal_quat.x;
    mposestamp_.pose.orientation.y = goal_quat.y;
    mposestamp_.pose.orientation.z = goal_quat.z;
    mposestamp_.pose.orientation.w = goal_quat.w;

    mpath_.poses.push_back(mposestamp_);
}

void SlamMapping::Visualization_Map(){
    // visualization_msgs::MarkerArray Map_RedCones_;
    Map_RedConeArray.markers.clear();
    Map_BlueConeArray.markers.clear();

    //红色锥桶可视化
    Map_RedCone.header.frame_id = "map";
    Map_RedCone.header.stamp = ros::Time::now();
    Map_RedCone.ns = "red";
    Map_RedCone.action = visualization_msgs::Marker::ADD;
    Map_RedCone.type = visualization_msgs::Marker::SPHERE;
    Map_RedCone.scale.x = 0.5;
    Map_RedCone.scale.y = 0.5;
    Map_RedCone.scale.z = 0.5;
    Map_RedCone.color.a = 0.5;
    Map_RedCone.color.r = 255;
    Map_RedCone.color.g = 0;
    Map_RedCone.color.b = 0;
    Map_RedCone.color.a = 1.0;
    
    int i=0;
    for(const auto &iter: map_.cone_red) {
        Map_RedCone.pose.position.x = iter.x;
        Map_RedCone.pose.position.y = iter.y;
        Map_RedCone.id = i;
        i++;
        //Map_RedCone.points.push_back(p);
        Map_RedConeArray.markers.push_back(Map_RedCone);
    }

    //蓝色锥桶可视化
    Map_BlueCone.header.frame_id = "map";
    Map_BlueCone.header.stamp = ros::Time::now();
    Map_BlueCone.ns = "blue";
    Map_BlueCone.action = visualization_msgs::Marker::ADD;
    Map_BlueCone.type = visualization_msgs::Marker::SPHERE;
    Map_BlueCone.scale.x = 0.5;
    Map_BlueCone.scale.y = 0.5;
    Map_BlueCone.scale.z = 0.5;
    Map_BlueCone.color.a = 0.5;
    Map_BlueCone.color.r = 0;
    Map_BlueCone.color.g = 0;
    Map_BlueCone.color.b = 255;
    Map_BlueCone.color.a = 1.0;
    
    int j=0;
    for(const auto &iter: map_.cone_blue) {
        Map_BlueCone.pose.position.x = iter.x;
        Map_BlueCone.pose.position.y = iter.y;
        Map_BlueCone.id = j;
        j++;
        Map_BlueConeArray.markers.push_back(Map_BlueCone);
    }

    //LocalMap可视化
    LocalMap_ConeArray.markers.clear();
    LocalMap_Cone.header.frame_id = "map";
    LocalMap_Cone.header.stamp = ros::Time::now();
    LocalMap_Cone.ns = "green";
    LocalMap_Cone.action = visualization_msgs::Marker::ADD;
    LocalMap_Cone.type = visualization_msgs::Marker::SPHERE;
    LocalMap_Cone.scale.x = 0.2;
    LocalMap_Cone.scale.y = 0.2;
    LocalMap_Cone.scale.z = 0.2;
    LocalMap_Cone.color.a = 0.5;
    LocalMap_Cone.color.r = 0;
    LocalMap_Cone.color.g = 255;
    LocalMap_Cone.color.b = 0;
    LocalMap_Cone.color.a = 1.0;
    LocalMap_Cone.lifetime=ros::Duration(0.5);

    int k=0;
    for(const auto &iter: localmap_.cone_blue) {
        LocalMap_Cone.pose.position.x = iter.x;
        LocalMap_Cone.pose.position.y = iter.y;
        LocalMap_Cone.id = k;
        k++;
        LocalMap_ConeArray.markers.push_back(LocalMap_Cone);
    }
    for(const auto &iter: localmap_.cone_red) {
        LocalMap_Cone.pose.position.x = iter.x;
        LocalMap_Cone.pose.position.y = iter.y;
        LocalMap_Cone.id = k;
        k++;
        LocalMap_ConeArray.markers.push_back(LocalMap_Cone);
    }

    //Neighbor锥桶可视化
    Neighbor_Cone.header.frame_id = "map";
    Neighbor_Cone.header.stamp = ros::Time::now();
    Neighbor_Cone.ns = "yellow";
    Neighbor_Cone.action = visualization_msgs::Marker::ADD;
    Neighbor_Cone.type = visualization_msgs::Marker::SPHERE;
    Neighbor_Cone.scale.x = 0.4;
    Neighbor_Cone.scale.y = 0.4;
    Neighbor_Cone.scale.z = 0.4;
    Neighbor_Cone.color.a = 0.5;
    Neighbor_Cone.color.r = 255;
    Neighbor_Cone.color.g = 255;
    Neighbor_Cone.color.b = 0;
    Neighbor_Cone.color.a = 1.0;
    Neighbor_Cone.lifetime=ros::Duration(0.5);
    
    int l=0;
    for(const auto &iter: neighbormap_.cone_blue) {
        Neighbor_Cone.pose.position.x = iter.x;
        Neighbor_Cone.pose.position.y = iter.y;
        Neighbor_Cone.id = l;
        l++;
        Neighbor_ConeArray.markers.push_back(Neighbor_Cone);
    }
    for(const auto &iter: neighbormap_.cone_red) {
        Neighbor_Cone.pose.position.x = iter.x;
        Neighbor_Cone.pose.position.y = iter.y;
        Neighbor_Cone.id = l;
        l++;
        Neighbor_ConeArray.markers.push_back(Neighbor_Cone);
    }


    // ROS_INFO("Visualization_Map Successful");
}

//lapCount用来识别车辆经过第几圈，具体策略是利用定位来确认
//odom是200Hz，Lidar是10Hz
void SlamMapping::lapCount_Calculate(const Mapping::OrientedPoint &odom_pose_){
    //lapCount_number帧数小于200的话，就不进行lapCount计算
    lapCount_number_ ++;
    int lapCount_thershold = 200;
    if(lapCount_number_ <= lapCount_thershold){
        ROS_INFO("lapCount_Calculate default lapCount_number is %d, we have %d laps",lapCount_number_, lapCount_.data);
        return;
    }

    Mapping::OrientedPoint base_pose;
    base_pose.x = StartPose.x; 
    base_pose.y = StartPose.y;
    base_pose.theta = StartPose.theta;
    double dist_to_basepoint = hypot(base_pose.x - odom_pose_.x, base_pose.y - odom_pose_.y);
    //车辆与原点距离小于0.1m，则认为在原点附近
    double dist_thershold = 4;
    ROS_INFO("we are now at (%lf, %lf) , dist_to_basepoint is %lf", odom_pose_.x, odom_pose_.y, dist_to_basepoint);

    if(dist_to_basepoint <= dist_thershold){
        lapCount_.data ++;
        // ROS_INFO("now lapCount = %d", lapCount_.data);
        //车辆进过一圈后，lapCount_number_清0
        lapCount_number_ = 0;
    }
    ROS_INFO("lapCount_Calculate Successful, we have %d laps", lapCount_.data);
}

/*激光信息的回调函数，每次节点收到一条消息时都将调用此函数，这个函数也是主要函数*/
void SlamMapping::laserCallback(const fsd_common_msgs::Map::ConstPtr& laserCloudMsg)
{
    // if(lapCount_.data!=0 && laser_number_now >= 50){
    //     ROS_INFO("LaserCallback  doesn't work now");
    //     return;
    // }

    laser_number_++;
    laser_number_now++;

    ScanCones_red.clear();
    ScanCones_blue.clear();
    // ScanCones_yellow.clear();
    // ScanCones_unknow.clear();

    //拷贝当前扫描数据到各类型的ScanCones中
    for(int i = 0;i<laserCloudMsg -> cone_red.size();i++){
        if(laserCloudMsg -> cone_red[i].position.y > -1 && laserCloudMsg -> cone_red[i].position.x < 5){
            ScanCones_red.push_back(laserCloudMsg -> cone_red[i]);
        }
    }

    for(int i = 0;i<laserCloudMsg -> cone_blue.size();i++){
        if(laserCloudMsg -> cone_blue[i].position.y < 1 && laserCloudMsg -> cone_blue[i].position.x < 6){
            ScanCones_blue.push_back(laserCloudMsg -> cone_blue[i]);
        }
    }

    // for(i = 0;i<laserCloudMsg -> cone_yellow.size();i++){
    //     ScanCones_yellow.push_back(laserCloudMsg -> cone_yellow[i]);
    // }

    // for(i = 0;i<laserCloudMsg -> cone_unknow.size();i++){
    //     ScanCones_unknow.push_back(laserCloudMsg -> cone_unknow[i]);
    // }
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
void SlamMapping::UpdateMap(const ros::Time& t, const  std::vector<fsd_common_msgs::Cone> &ScanCones, std::vector<fsd_common_msgs::Cone> &UpdateCones, std::vector<fsd_common_msgs::Cone> &HistoryCones)
{
    // ROS_DEBUG("Update map");
    //做已知定位的负责建图，Vehicle_pose表示当前车辆位姿
    Mapping::OrientedPoint Vehicle_pose;
    Vehicle_pose.x = carstate_.car_state.x;
    Vehicle_pose.y = carstate_.car_state.y;
    Vehicle_pose.theta = carstate_.car_state.theta;
    //getVehiclePose(Vehicle_pose, t);

    //x轴纵向，y轴横向，z轴朝上
    double mx , my;
    fsd_common_msgs::Cone updateCone;
    if (ScanCones.size() == 0){
        ROS_INFO("UpdateCones default Successful");
        return;
    }
    else{
        updateCone.color = ScanCones[0].color;
    }
    
    for (int i=0;i<ScanCones.size();i++){
        mx = ScanCones[i].position.x;
        my = ScanCones[i].position.y;
        updateCone.position.x = Vehicle_pose.x + mx * cos(Vehicle_pose.theta) - my * sin(Vehicle_pose.theta);
        updateCone.position.y = Vehicle_pose.y + mx * sin(Vehicle_pose.theta) + my *cos(Vehicle_pose.theta);
        UpdateCones.push_back(updateCone);
        HistoryCones.push_back(updateCone);
        if(updateCone.position.x > 200){
            ROS_INFO("UpdateCones OutOfDist");
        }
    }
    // ROS_INFO("UpdateCones Successful");
}

void SlamMapping::generate_CenterCone(std::vector<fsd_common_msgs::Cone> &mCenterCone, std::vector<fsd_common_msgs::Cone> &UpdateCones){
    //激光第一帧的一次聚类点全算中心点
    //输入当前扫描到的laserCloudMsg.cone
    if(UpdateCones.size() == 0){
        ROS_INFO("generate_CenterCone default Successful");
        return;
    }

    if(laser_number_ == 1){
        for(int i =0;i<UpdateCones.size();i++){
            mCenterCone.push_back(UpdateCones[i]);
        }
    }

    else{
        double thershold_dist = 1.5;//聚类距离改为1.5m
        bool isCenter = true;
        for(int i = 0;i<UpdateCones.size();i++){
            for(int j = 0;j<mCenterCone.size();j++){
                double dist = Caculate_Dist(mCenterCone[j], UpdateCones[i]);
                if(dist < thershold_dist){
                    isCenter = false;
                }               
            }
            if(isCenter){
                mCenterCone.push_back(UpdateCones[i]);
            }
        }
    }
    // ROS_INFO("generate_CenterCone Successful");
}

//计算两个锥桶间的欧式距离
double SlamMapping::Caculate_Dist(const fsd_common_msgs::Cone &Cone1, const fsd_common_msgs::Cone &Cone2){
    double distance;
    geometry_msgs::Point p1 = Cone1.position;
    geometry_msgs::Point p2 = Cone2.position;
    distance = hypot(p1.x-p2.x, p1.y-p2.y);
    return distance;
}

/*二次聚类，输入为CalibratedMapPoints和当前簇数*/
void SlamMapping::SecondCluster(std::vector<fsd_common_msgs::Cone> &mClusterCones, const std::vector<fsd_common_msgs::Cone> &CenterCones, std::vector<fsd_common_msgs::Cone> &UpdateCones, std::vector<fsd_common_msgs::Cone> &HistoryCones){
    //按照当前的传入数据点，判断一共有多少簇，初始化K个中心点
    //每一帧，SortOutCones都要清空一次，重新定义一遍
    //label从0到K，代表K个簇

    int label_cluster = 0;
    K = CenterCones.size();
    // ROS_INFO("CenterCones has %d Class", K);
    mClusterCones.clear();
    
    if(UpdateCones.size()==0){
        ROS_INFO("SecondCluster UpdateCones default Successful");
        return;
    }
    else{
        if(K == 0){
            ROS_INFO("SecondCluster CenterCones default Successful");
            return;
        }
    }
    fsd_common_msgs::Cone SortCone;
    std::vector<std::vector<fsd_common_msgs::Cone>> SortOutCones(K);

    //如何将SortOutCones下的锥桶进行归类，
    SortCone.color = UpdateCones[0].color;

    //遍历UpdateCones上的点，进行归类，获得label_cluster，根据label_cluster，装进SortOutCones中
    for(int i =0; i < UpdateCones.size(); i++){
        label_cluster = clusterOfMap(CenterCones, UpdateCones[i]);
        SortCone.position.x = UpdateCones[i].position.x;
        SortCone.position.y = UpdateCones[i].position.y;
        SortCone.position.z = 0;
        //怎么把分好类的label的UpdateCones，push到SortOutCones里
        SortOutCones[label_cluster].push_back(SortCone);
    }

    for(int i=0;i<HistoryCones.size();i++){
        label_cluster = clusterOfMap(CenterCones, HistoryCones[i]);
        SortCone.position.x = HistoryCones[i].position.x;
        SortCone.position.y = HistoryCones[i].position.y;
        SortCone.position.z = 0;
        //怎么把分好类的label的HistoryCones，push到SortOutCones里
        SortOutCones[label_cluster].push_back(SortCone);
        }

    //根据SortOutCones中每一个簇，归纳出一个中心点
    getClusterPoints(SortOutCones, mClusterCones);

    //到这里，ClusterMapPoints肯定是聚类完了，而且应该是各点的均值，含有label值
    got_clustered_ = true;
    // ROS_INFO("SecondCluster Successful");
}

//根据质心，决定当前点属于哪个簇
int SlamMapping::clusterOfMap(const std::vector<fsd_common_msgs::Cone> &CenterCones, const fsd_common_msgs::Cone &mUpdateCones) {
	float dist = Caculate_Dist(CenterCones[0], mUpdateCones);
	float tmp;
	int label = 0;
	for (int i = 1; i < CenterCones.size(); i++) {
		tmp = Caculate_Dist(CenterCones[i], mUpdateCones);
		if (tmp<dist) { dist = tmp; label = i; }
	}
	return label;
}

//取出各簇的点的平均值
void SlamMapping::getClusterPoints(std::vector<std::vector<fsd_common_msgs::Cone>> SortOutCones, std::vector<fsd_common_msgs::Cone> &mClusterCones){
    double x_ave;
    double y_ave;
    fsd_common_msgs::Cone AverageCone;

    if(SortOutCones.size()==0){
        ROS_INFO("getClusterPoints No.1 default Successful"); 
        return;
    }
    else{
        for(int i=0;i<SortOutCones.size();i++){
            if(SortOutCones[i].size() > 0){
                AverageCone.color = SortOutCones[i][0].color;
            }
        }
    }
    
    for(int i=0;i<K;i++ ){
        double x_sum=0;
        double y_sum=0;
        if(SortOutCones[i].size() >= 1){
            for(int j=0;j<SortOutCones[i].size();j++){
                x_sum += SortOutCones[i][j].position.x;
                y_sum += SortOutCones[i][j].position.y;
            }
            x_ave = x_sum / SortOutCones[i].size();
            y_ave = y_sum / SortOutCones[i].size();
            AverageCone.position.x = x_ave;
            AverageCone.position.y = y_ave;
            AverageCone.position.z = 0;
            mClusterCones.push_back(AverageCone);            
        }
    }
    // ROS_INFO("getClusterPoints Successful");  
        
}



/*一些没人要的功能模块
(1)线性插值
    //接下来进行index_min的插值计算，重点是要关注正负向最小点的关系
    // std::vector<geometry_msgs::Point> Insert_point;
    // for(int i=1;i<4;i++){
    //     geometry_msgs::Point Insert_relate_point;
    //     double rx_plus = mapcones[index_min_minor].x;
    //     double ry_plus = mapcones[index_min_minor].y;
    //     double rx_minor = mapcones[index_min_plus].x;
    //     double ry_minor = mapcones[index_min_plus].y;
    //     //插值公式在哪里？
    //     Insert_relate_point.x = (rx_plus+rx_minor)*i/4;
    //     Insert_relate_point.y = (ry_plus+ry_minor)*i/4;
    //     Insert_point.push_back(Insert_relate_point);
    // }
        if(index_min_insert_notready){
            ROS_INFO("index_min_notready is true");
            return;
        }

    // double dist_insert = 999;
    // double dist_base_insert;
    // int dist_min_insert;
    // bool index_min_insert_notready = true;
    // for(int i=0;i<Insert_point.size();i++){
    //     dist_base_insert = hypot(Insert_point[i].x, Insert_point[i].y);
    //         if(dist_base_insert < dist_insert){
    //             dist_insert = dist_base_insert;
    //             dist_min_insert = i;
    //             index_min_insert_notready = false;
    //         }
    // }

(2)以距离范围为判据的localmap生成
    //以下是以距离范围内点均作为localmap点为判据
    // for(int i =0;i<basemap_.size();i++){
    //     x_now = basemap_[i].x;
    //     y_now = basemap_[i].y;
    //     x_relate = basemap_[index_min_plus].x;
    //     y_relate = basemap_[index_min_plus].y;

    //     if( x_now > x_relate && hypot(x_now - x_relate, y_now - y_relate) <= dist_thershold){
    //         index_min_plus = i;
    //         mlocalcones.push_back(mapcones[index_min_plus]);
    //     }
    // }

(3)线性插值用的localmap
    //首先将全局地图，全部转换到base_link中，然后基于base_link中的点，找到前视的10个点，将其输出
    //基本原理是，判断点的纵向x是否大于0，然后是否为距离mpose最近的十个点，然后push到localmap_
    Mapping::Point mpoint;
    geometry_msgs::Point point_base_link;
    std::vector<geometry_msgs::Point> basemap_;
    double dist_min_plus = 999;
    double dist_min_minor = 999;
    double dist_base_plus;
    double dist_base_minor;
    //plus正向最近点、minor负向最近点
    int index_min_plus;
    int index_min_minor;
    bool index_min_plus_notready = true;
    bool index_min_minor_notready = true;
    bool flag_relate_base = false;

    for(int i =0;i<mapcones.size();i++){
        mpoint.x = mapcones[i].x;
        mpoint.y = mapcones[i].y;
        Points_map_to_vehicle(npose, mpoint);
        point_base_link.x = mpoint.x;
        point_base_link.y = mpoint.y;
        //将当前map坐标系下的点全转到base_link下,索引与map_相同
        basemap_.push_back(point_base_link);

        //找当前车辆位置的对应点(正向)的索引
        if(mpoint.x >=0){
            dist_base_plus = hypot(mpoint.x, mpoint.y);
            if(dist_base_plus < dist_min_plus){
                dist_min_plus = dist_base_plus;
                index_min_plus = i;
                index_min_plus_notready = false;
            }
        }

        //找当前车辆位置对应点(负向)的索引
        if(mpoint.x <0){
            dist_base_minor = hypot(mpoint.x, mpoint.y);
            if(dist_base_minor < dist_min_minor){
                dist_min_minor = dist_base_minor;
                index_min_minor = i;
                index_min_minor_notready = false;
            }
        }
    }
    ROS_INFO("after Loop Successful");

    if(index_min_plus_notready || index_min_minor_notready){
        ROS_INFO("index_min_notready is true");
        return;
    }

    //利用斜率计算前进插点，y-y0 = k * (x - x0)
    if(basemap_[index_min_minor].x - basemap_[index_min_plus].x != 0 && basemap_[index_min_plus].x != 0){
        double rate_ = (basemap_[index_min_minor].y - basemap_[index_min_plus].y) / (basemap_[index_min_minor].x - basemap_[index_min_plus].x);
        double x_ = 0;
        double y_ = rate_ * (-basemap_[index_min_plus].x) + basemap_[index_min_plus].y;
        //首先将相关点塞进localmap
        geometry_msgs::Point minsertlocalpoint;
        Mapping::Point minsertpoint;
        minsertpoint.x = x_;
        minsertpoint.y = y_;
        Points_vehicle_to_map(npose, minsertpoint);
        minsertlocalpoint.x = minsertpoint.x;
        minsertlocalpoint.y = minsertpoint.y;
        mlocalcones.push_back(minsertlocalpoint);
    }

    //再把正向最小点放进去,先检验正向最小点与前插点是否重合
    mlocalcones.push_back(mapcones[index_min_plus]);

    ROS_INFO("after related Successful");
    //找到相关点，循环取当前相关点对应的邻近点
    int cone_range = 6; //表示localmap顺序传递6个点
    double x_now, y_now;
    double x_relate, y_relate;
    double dist_thershold = 6;
    
    for(int j=0;j<cone_range - 1;j++){
        bool HavenotFoundNeighbor = true;
        int index_min_before = index_min_plus;

        for(int i =0;i<basemap_.size();i++){
            x_now = basemap_[i].x;
            y_now = basemap_[i].y;
            x_relate = basemap_[index_min_plus].x;
            y_relate = basemap_[index_min_plus].y;

            //想办法在两点间把值插进去,可以设插值点的数量不唯一，目前插1个点
            if(HavenotFoundNeighbor && x_now > x_relate){
                if(hypot(x_now - x_relate, y_now - y_relate) <= dist_thershold){
                    
                    geometry_msgs::Point Interpolating_point;
                    Mapping::Point midpoint;

                    midpoint.x = (x_now+x_relate) / 2;
                    midpoint.y = (y_now+y_relate) / 2;

                    Points_vehicle_to_map(npose, midpoint);
                    Interpolating_point.x = midpoint.x;
                    Interpolating_point.y = midpoint.y;                  
                    mlocalcones.push_back(Interpolating_point);

                    index_min_plus = i;
                    HavenotFoundNeighbor = false;
                    mlocalcones.push_back(mapcones[index_min_plus]);
                }
            }
        }        
    }

(4)
    for(int j=0;j<5;j++){
        bool HavenotFoundNeighbor = true;

        for(int i =0;i<basemap_.size();i++){
            x_now = basemap_[i].x;
            y_now = basemap_[i].y;
            x_relate = basemap_[index_min_plus].x;
            y_relate = basemap_[index_min_plus].y;

            //判定条件
            if(HavenotFoundNeighbor && x_now > x_relate){
                if(hypot(x_now - x_relate, y_now - y_relate) <= dist_thershold){
                    index_min_plus = i;
                    HavenotFoundNeighbor = false;
                    relate_point++;
                    if(relate_point%2 == 1){
                        //考虑是否是因为点排序的问题
                        wx.push_back(x_now);
                        wy.push_back(y_now);

                    }                    
                }
            }
        }
    }
*/