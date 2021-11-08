#include <iostream>
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Vector3.h>
#include <fsd_common_msgs/Map.h>

#include "fsd_common_msgs/CarState.h"

class Slam_interface{
    private:
    	
        ros::Publisher PathPublisher_ ;
        ros::Publisher AccPublisher_ ;
        ros::Publisher SpeedPublisher_;
        ros::Publisher ClusterPublisher_ ;

        ros::Subscriber StateSubscriber_;
	    ros::Subscriber PCloudSubscriber_ ;
        

    public:
        void CarStateCallback(const fsd_common_msgs::CarState &mstate);
        void PCloundCallback(const fsd_common_msgs::Map& mpcloud);
        void Msg_to_Pub();
        void runAlgorithm();

        ros::NodeHandle nh_;
        geometry_msgs::Pose2D Car_Pose;
<<<<<<< HEAD
        geometry_msgs::Vector3 Car_Accel;
=======
        geometry_msgs::Accel Car_Accel;
>>>>>>> ff1b3416472d3deea21f69eefd306ff96578adad
        geometry_msgs::Vector3 Car_Speed;
        fsd_common_msgs::Map First_Cluster_Point;

};