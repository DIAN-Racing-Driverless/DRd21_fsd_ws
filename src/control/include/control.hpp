/*
    Formula Student Driverless Project (FSD-Project).
    Copyright (c) 2021:
     - chentairan <tairanchen@bitfsd.cn>

    FSD-Project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    FSD-Project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with FSD-Project.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef CONTROL_HPP
#define CONTROL_HPP

#include "Utils/types.h"
#include "Utils/param.h"
#include "Utils/visual.h"

#include "Track/track_base.h"
#include "Track/trackdrive_track.h"
#include "Track/line_track.h"
#include "Track/skidpad_track.h"

#include "Solver/solver_base.h"
#include "Solver/mpc_solver.h"
#include "Solver/pure_pursuit_solver.h"

namespace ns_control {

class Control {

 public:
  Control(ros::NodeHandle &nh);

  void runAlgorithm();

  void setTransMat(const std_msgs::Float64MultiArray &msgs);
  void setEndPoint(const geometry_msgs::Point &msgs);
  void setMap(const fsd_common_msgs::Map &msgs);
  void setCarState(const fsd_common_msgs::CarState &msgs);
  visualization_msgs::MarkerArray getRefPath();
  visualization_msgs::MarkerArray getPrePath();
  visualization_msgs::MarkerArray getMapPath();

  fsd_common_msgs::ControlCommand getCmd();

  visualization_msgs::MarkerArray RefPath_;
  visualization_msgs::MarkerArray PrePath_;
  visualization_msgs::MarkerArray mapPath_;        //对于直线和八字显示规划好的轨迹


 private:

  bool Check();
  void setTrack();

 private:

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
  Trajectory map_traj_;

  bool is_init = false;
};
}

#endif //CONTROL_HPP
