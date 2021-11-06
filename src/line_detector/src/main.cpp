#include <ros/ros.h>
#include <line_detector.h>
#include <sstream>

typedef ns_line_detector::LineDetector LineDetector;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "line_detector");
	ros::NodeHandle nh;
	LineDetector line_detector(nh);
	ros::Rate timer_hz(50);
	while(ros::ok())
	{
		line_detector.runAlgorithm();
		line_detector.sendMsg();
		timer_hz.sleep();
		ros::spinOnce();
	}
	return 0;
}