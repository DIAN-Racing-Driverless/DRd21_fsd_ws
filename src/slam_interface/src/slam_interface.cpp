#include "ros/ros.h"
#include "ros/console.h"

#include "fsd_common_msgs/CarState.h"
#include "fsd_common_msgs/CarStateDt.h"

#include "slam_interface.h"

void Slam_interface::runAlgorithm()
{

	PathPublisher_ = nh_.advertise<geometry_msgs::Pose2D>("/veh_status/pose", 1, true);
<<<<<<< HEAD
	AccPublisher_ = nh_.advertise<geometry_msgs::Vector3>("/veh_status/acc",1, true);
=======
	AccPublisher_ = nh_.advertise<geometry_msgs::Accel>("/veh_status/acc",1, true);
>>>>>>> ff1b3416472d3deea21f69eefd306ff96578adad
	SpeedPublisher_ = nh_.advertise<geometry_msgs::Vector3>("/veh_status/speed",1, true);
	ClusterPublisher_ = nh_.advertise<fsd_common_msgs::Map>("/perception/ps_interface",1, true);

    //将CarState转化为pose,acc和speed，然后发布出去
	StateSubscriber_ = nh_.subscribe("/estimation/slam/state", 1, &Slam_interface::CarStateCallback, this);
	PCloudSubscriber_ = nh_.subscribe("/perception/lidar_cluster", 1,&Slam_interface::PCloundCallback, this);

    std::cout <<"Slam_interface run!!!"<<std::endl;

}

void Slam_interface::CarStateCallback(const fsd_common_msgs::CarState& mstate){
    Car_Pose.x = mstate.car_state.x;
    Car_Pose.y = mstate.car_state.y;
    Car_Pose.theta = mstate.car_state.theta;

    Car_Speed.x = mstate.car_state_dt.car_state_dt.x;
    Car_Speed.y = mstate.car_state_dt.car_state_dt.y;
    Car_Speed.z = mstate.car_state_dt.car_state_dt.theta;

<<<<<<< HEAD
    Car_Accel.x = mstate.car_state_dt.car_state_a.x;
    Car_Accel.y = mstate.car_state_dt.car_state_a.y;
    Car_Accel.z = mstate.car_state_dt.car_state_a.theta;
=======
    Car_Accel.linear.x = mstate.car_state_dt.car_state_a.x;
    Car_Accel.linear.y = mstate.car_state_dt.car_state_a.y;
    Car_Accel.linear.z = 0;
    Car_Accel.angular.x = 0;
    Car_Accel.angular.y = 0;
    Car_Accel.angular.z = mstate.car_state_dt.car_state_a.theta;
>>>>>>> ff1b3416472d3deea21f69eefd306ff96578adad

}

void Slam_interface::PCloundCallback(const fsd_common_msgs::Map& mpcloud){
    First_Cluster_Point = mpcloud;
    Msg_to_Pub();
}

void Slam_interface::Msg_to_Pub(){
    PathPublisher_.publish(Car_Pose);
    AccPublisher_.publish(Car_Accel);
    SpeedPublisher_.publish(Car_Speed);
    ClusterPublisher_.publish(First_Cluster_Point);
}