#include "ros/ros.h"
#include "slam_interface.h"

int main(int argc, char **argv)
{
	ros::init(argc,argv,"slam_interface_node");
    Slam_interface Go_;
	Go_.runAlgorithm();
	ros::spin();
	return 0;
}