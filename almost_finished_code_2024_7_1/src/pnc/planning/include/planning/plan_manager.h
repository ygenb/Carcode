#ifndef PLAN_MANAGER_H
#define PLAN_MANAGER_H

#include <ros/ros.h>
#include <mapping/map_manager.h>
#include <vehicle_params/vehicle_param.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <car_msgs/CarTraj.h>
#include <car_msgs/CarTrajS.h>
#include <tf/tf.h>
#include <path_search/kino_astar.h>
#include <nav_msgs/Path.h>

#include "plan_utils/traj_container.hpp"
#include "plan_utils/poly_traj_utils.hpp"
#include "traj_opt/traj_optimizer.h"
#include "wrapper/config.h"

class PlanManager {
 public:
  PlanManager(ros::NodeHandle& nh);

 private:
  enum STATE
  {
    INIT,
    PLANNING,
    NEW_GOAL
  };
  Eigen::Vector4d cur_state_, goal_state_;
  bool has_odom_, planSuccess;

  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg);
  bool getKinoPath(Eigen::Vector4d &end_state);
  bool RunMINCO();
  void getRectangleConst(std::vector<Eigen::Vector3d> statelist);
  void displayPolyTraj(std::unique_ptr<plan_utils::SingulTrajData> &display_traj);
  void traj_server_callback(const ros::TimerEvent& event);
  void replan_callback(const ros::TimerEvent& event);
  void visualizeCarCmd(const car_msgs::CarTraj &msg, ros::Time timeStamp);
  bool plan();
  bool CheckReplan();
  void pub_traj();

  std::mutex m;
  double dt, N;  // MPC params
  double replan_freq;
  double traj_piece_duration_;
  double replan_error_d_, replan_error_theta_;
  int traj_res, dense_traj_res; 
  int final_traj_index_, exe_traj_index_;
  STATE exec_state_;

  mapping::MapManager::Ptr map_;  
  VehicleParam::Ptr vhc_param_;
  path_searching::KinoAstar::Ptr kino_path_finder_;
  plan_utils::KinoTrajData kino_trajs_;
  plan_utils::TrajContainer traj_container_;
  PolyTrajOptimizer::Ptr ploy_traj_opt_;
  std::unique_ptr<plan_utils::SingulTrajData> executing_traj_, next_traj_;

  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_, goal_sub_;
  ros::Publisher ctrl_pub_, dense_kino_pub_, cmd_vis_pub_, cmd_arrow_vis_;
  ros::Publisher Debugtraj1Pub, DebugCorridorPub;
  ros::Publisher traj_path_pub, wholebody_traj_pub;
  ros::Timer trajServerTimer, replanTimer;
  std::vector<Eigen::MatrixXd> hPolys_, display_hPolys_;
};
#endif