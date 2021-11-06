

#pragma once

namespace ns_control{

	
class Solver {
public:
	void setTrajectory(const Trajectory &trajectory);
	void setState(const VehicleState &state);
	Trajectory getTrajectory();
	Trajectory predictive_path;
	fsd_common_msgs::ControlCommand getCmd();
	virtual void solve() = 0;  

protected:
	Trajectory trajectory_;
	VehicleState state_;
	fsd_common_msgs::ControlCommand control_command_;
}; 

class MPC_Solver : public Solver {    
	public solve();   //get the predictive_path and the cmd   mpc solve

	double epsi = 0.0;
	double cte = 0.0;
};


class Track {
public:
	Track() = default;
	~Track() = default;

	virtual bool genTraj() = 0;
	virtual bool CalculateTraj(Tracjectory &refline) = 0;
	void setMap(const fsd_common_msgs::Map &map);
	void setState(const VehicleState &state);
	void setTransMat(const Eigen::Matrix4f &transPoint);
	void setEndPoint(const geometry_msgs::Point &endPoint);
protected:
	fsd_common_msgs::Map map_;
	VehicleState state_;
	Trajectory trajectory_;
	geometry_msgs::Point endPoint_;
	Eigen::Matrix4f transMat_;

}

class Autox_Track : public Track
{
public:
	bool genTraj();    // get the spline of trajectory 
	bool CalculateTraj(Tracjectory &refline);  //

}



class Control {
public:
	Control(ros::NodeHandle &nh);
	void runAlgorithm();

	void setTransMat(const std_msgs::Float64MultArray &msgs);
	void setEndPoint(const geometry_msgs::Point &msgs);
	void setMap(const fsd_common_msgs::Map &msgs);
	void setCarState(const fsd_common_msgs::CarState &msgs);
	visualization_msgs::MarkerArray getRefPath();
	visualization_msgs::Array getPrePath();

	fsd_common_msgs::ControlCommand getCmd();

	visualization_msgs::MarkerArray RefPath_;
	visualization_msgs::MarkerArray PrePath_;
private:
	bool check();
	void setTrack();
	ros::NodeHandle &nh_;
	std::string mission_;
	std::string controller_;

	Track *track_;
	Autox_Track trackdrive_track_;
	Line_Track line_track_;
	Skidpad_Track skidpad_track_;

	Solver *solver_;
	MPC_Solver mpc_solver_;
	Pure_Pursuit_Solver pure_pursuit_solver_;

	geometry_msgs::Point endPoint_;
	Eigen::Matrix4f transMat_;
	fsd_common_msgs::Map local_map_;
	fsd_common_msgs::CarState car_state_;
	fsd_common_msgs::ControlCommand cmd_;

	Trajectory refline_;
	bool is_init = false;
};

class ControlHandle {
public:
	ControlHandle(ros::NodeHandle &nodeHandle);
	int getNodeRate() const;

	void loadParameters();
	void subscribeToTopics();
	void publishToTopics();
	void run();
	void sendMsg();

private:
	void localMapCallback(const fsd_common_msgs::Map &map);
	void carStateCallback(const fsd_common_msgs::CarState &msg);
	void transMatCallback(const std_msgs::Float64MultArray &msgs);
	void endPointCallback(const geometry_msgs::Point &msgs);

	ros::NodeHandle nodeHandle_;
	ros::Subsciber localMapSubscriber_;
	ros::Subscriber transMatSubscriber_;
	ros::Subscriber endPointSubscriber_;
	ros::Subscriber carStateSubscriber_;

	ros::Publisher cmdPublisher_;
	ros::Publisher refPathPublisher_;
	ros::Publisher prePathPublisher_;

	std::string car_sate_topic_name_;
	std::string transform_matrix_topic_name_;
	std::string end_point_topic_name_;
	std::string map_topic_name_;
	std::string ctrl_cmd_topic_name_;
	std::string predict_path_topic_name_;
	std::string ref_path_topic_name_;

	int node_rate_;
	Control control_;

};



bool Control::setTrack() {
	if(mission_ == "trackdrive") {
		track_->setMap(local_map_);
		track_->genTraj();
	}
}

void Control::runAlgorithm(){
	setTrack();

	track_->setState(VehicleState(car_state_,cmd_));
	track_->CalculateTraj(refline_)

	solver_setState(VehicleState(car_state_,cmd_));
	solver_->setTrajectory(refline_);
	solver_->solve();    //MPC solve()

	cmd_ = solver_->getCmd();

}



void Solver::setState(const VehicleState &state){state_ = state;}
void Solver::setTrajectory(const Trajectory &trajectory) {
	trajectory_ = trajectory;
}
fsd_common_msgs::ControlCommand Solver::getCmd() {
  return control_command_;
}



void Track::setMap(const fsd_common_msgs::Map &map) { map_ = map; }
void Track::setState(const VehicleState &state) { state_ = state; }
void Track::setEndPoint(const geometry_msgs::Point &endPoint) { endPoint_ = endPoint; }
void Track::setTransMat(const Eigen::Matrix4f &transMat) { transMat_ = transMat; }


}; //namespace ns_control


for(int i=0;i<trajectory_.size();i++){
	double delta_x = trajectory_[i].pts.x-px;
	double delta_y = trajectory_[i].pts.y - py;
	double dist = std::hypot(delta_x, delta_y);
	if(dist < min) {
		min = dist;
		index_min = i;
	}
}

if(index_min<0)
	return false;

const double desired_velocity = param_.desire_vel;
const int N = param_.N;
const double dt = param_.dt;
TrajectoryPoint tmp;

double v= std::fmax(state_.v,param_.initial_velocity);

double s_tmp = 0;

for(int i=0;i<N;i++){

}



param_