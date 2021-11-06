#include <ros/ros.h>
#include <skidpad_detector.h>
#include <sstream>

typedef ns_skidpad_detector::SkidpadDetector SkidpadDetector;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "skidpad_detector");
	ros::NodeHandle nh;
	SkidpadDetector skidpad_detector(nh);
	ros::Rate timer_hz(50);
	while(ros::ok())
	{
		skidpad_detector.runAlgorithm();
		skidpad_detector.sendMsg();
		timer_hz.sleep();
		ros::spinOnce();
	}
	return 0;
}