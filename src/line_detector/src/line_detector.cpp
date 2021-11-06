
#include <ros/ros.h>
#include "line_detector.h"

namespace ns_line_detector
{

LineDetector::LineDetector(ros::NodeHandle &nh)
{
	nh_=nh;
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
}

void LineDetector::sendMsg()
{
	  endPointPublisher_.publish(end_point_);
}


void LineDetector::lidarClusterCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	pcl::fromROSMsg(*msg,*cluster_ptr_);
	//ROS_INFO("lidar size is %ld",cluster_ptr_->points.size());

}

void LineDetector::runAlgorithm() {
    if(!getPath)
        createPath();
    else
        return;
}


void LineDetector::createPath() {
    if(cluster_ptr_->points.size() == 0)
        return;
    int accumulator[180][201]={0};
    double p,p1,p2,Y_right,Y_left;
    int theta1,theta2;
    for(int i=0; i<cluster_ptr_->points.size();i++)
    {
        if(cluster_ptr_->points[i].y > 4 || cluster_ptr_->points[i].y < -4)//raw is 2
            continue;
        for (int j=0; j<180; j++)
        {
            p=(cluster_ptr_->points[i].x * cos(j * M_PI / 180)+cluster_ptr_->points[i].y*sin(j * M_PI / 180))*5;
            if(p > 100)
                p = 100;
            accumulator[j][(int)p+100]+=1;            
       }
    }

    int max1 = 0;
    int max2 = 0;

    for(int i = 90 - allow_angle_error; i < 90 + allow_angle_error; i++)
    {
        for(int j = 0; j < 100; j++)
        {
            if(accumulator[i][j] >= max1)
            {
                max1 = accumulator[i][j];
                p1=((float)j-100)/5;
                theta1=i;
            }
        }
    }
   
    for(int i = 90 - allow_angle_error; i < 90 + allow_angle_error; i++)
    {
        for(int j = 100; j < 200; j++)
        {
            if(accumulator[i][j] >= max2)
            {
                max2 = accumulator[i][j];
                p2=((float)j-100)/5;
                theta2=i;
            }
        }
    }

    if (theta1==theta2)
	{
		if  (fabs(p1)<3 && fabs(p2)<3 )
        {
            getPath=true;
            std::cout<<"find ideal path"<<std::endl;

            Y_right = (p1-path_length*cos((float)theta1*M_PI/180.0))/sin((float)theta1*M_PI/180.0);
            Y_left = (p2-path_length*cos((float)theta2*M_PI/180.0))/sin((float)theta2*M_PI/180.0);
            end_point_.x = path_length;
            end_point_.y = -(Y_left + Y_right)/2;
            ROS_INFO("end_point is (x=%lf,y=%lf)",end_point_.x,end_point_.y);
            ROS_INFO("starting sending...");
        }
	}
    else
    {
        double check_x=(p1*cos((float)theta2*M_PI/180.0)-p2*cos((float)theta1*M_PI/180.0))/(sin((float)theta1*M_PI/180.0)*cos((float)theta2*M_PI/180.0)-sin((float)theta2*M_PI/180.0)*cos((float)theta1*M_PI/180.0));
        if ((check_x > 200 || check_x < -200)&&(fabs(p1)<3 && fabs(p2)<3))//直线距离车小于3米
        {
            getPath=true;
			std::cout<<"find path"<<std::endl;
            Y_right = (p1-path_length*cos((float)theta1*M_PI/180.0))/sin((float)theta1*M_PI/180.0);
            Y_left = (p2-path_length*cos((float)theta2*M_PI/180.0))/sin((float)theta2*M_PI/180.0);

            end_point_.x = path_length;
            end_point_.y = (Y_left + Y_right)/2;
            ROS_INFO("end_point is (x=%lf,y=%lf)",end_point_.x,end_point_.y);
            ROS_INFO("starting sending...");            
        }
        else
        {
            getPath=false;
            return;
        }
    }

}



} //end of namespace ns_line_detector