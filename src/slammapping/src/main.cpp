/*
slam_mapping
 */

/* Author: HoGinhang  */

#include <ros/ros.h>

#include "Fortest_DemoSlamMapping.h"

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "slam_mapping");

  SlamMapping gotest;
  
  ros::Rate timer_hz(10);
	while(ros::ok())
	{
		gotest.startLiveSlam();
    gotest.run_alogrithm();
    gotest.Visualization_Path();
    gotest.Visualization_Map();    //map可视化处理
    gotest.Msg_to_Pub();    //发布消息
		timer_hz.sleep();
		ros::spinOnce();
	}
  // ros::spin();

  return(0);
}

