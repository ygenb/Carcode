#include <mpc_car/mpc_car.hpp>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <cmath>
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


std::shared_ptr<mpc_car::MpcCar> mpcPtr_;
ros::Timer plan_timer_;
ros::Subscriber odom_sub_, traj_sub_, Imu_propagate_sub;
ros::Publisher vel_cmd_pub_,error_pub_, cmdPub, predPub; //add error_pub 推送error 
ros::Publisher state_pub_;
mpc_car::VectorX state_;
bool has_odom_ = false;
bool has_traj_ = false;
double delay_ = 0.0;
int controller_;
double traj_duration_;
ros::Time traj_start_time_;

int n = 0;
double dt = 0;
double odom_x = 0, odom_y = 0, odom_phi = 0; 
float traj_x = 0, traj_y = 0, traj_phi = 0;
double x_error = 0, y_error = 0, phi_error = 0,dis = 0;

void visualizeCarCmd(const std::vector<Eigen::Vector2d> points, ros::Time timeStamp)
{
    visualization_msgs::Marker Marker;
    Marker.id = 0;
    Marker.type = visualization_msgs::Marker::SPHERE_LIST;
    Marker.header.stamp = timeStamp;
    Marker.header.frame_id = "world";
    Marker.pose.orientation.w = 1.00;
    Marker.action = visualization_msgs::Marker::ADD;
    Marker.ns = "CarCmd";
    Marker.color.r = 1.00;
    Marker.color.g = 0.00;
    Marker.color.b = 0.00;
    Marker.color.a = 1.00;
    Marker.scale.x = 0.20;
    Marker.scale.y = 0.20;
    Marker.scale.z = 0.20;

    for (u_int i = 0; i < points.size(); i++)
    {
        geometry_msgs::Point point;
        point.x = points[i].x();
        point.y = points[i].y();
        point.z = 0.0;
        Marker.points.push_back(point);
    }
    cmdPub.publish(Marker);
}

void visualizePredictPath(const std::vector<Eigen::Vector2d> points, ros::Time timeStamp)
{
    visualization_msgs::Marker Marker;
    Marker.id = 0;
    Marker.type = visualization_msgs::Marker::SPHERE_LIST;
    Marker.header.stamp = timeStamp;
    Marker.header.frame_id = "world";
    Marker.pose.orientation.w = 1.00;
    Marker.action = visualization_msgs::Marker::ADD;
    Marker.ns = "predpath";
    Marker.color.r = 0.00;
    Marker.color.g = 1.00;
    Marker.color.b = 0.00;
    Marker.color.a = 1.00;
    Marker.scale.x = 0.20;
    Marker.scale.y = 0.20;
    Marker.scale.z = 0.20;

    for (u_int i = 0; i < points.size(); i++)
    {
        // if(i > 0)
        // {
        //   if((points[i+1] - points[i]).norm() < 1e-3)
        //     continue;
        // }
        geometry_msgs::Point point;
        point.x = points[i].x();
        point.y = points[i].y();
        point.z = 0.0;
        Marker.points.push_back(point);
    }
    // cmdPub.publish(Marker);
    predPub.publish(Marker);
}

void plan_timer_callback(const ros::TimerEvent& event) {
  if (has_odom_ && has_traj_) {
    // @NOTE check exceed traj duration
    double time_from_start = ros::Time::now().toSec() - traj_start_time_.toSec();
    if(time_from_start > traj_duration_ - delay_) {
      ROS_INFO_ONCE("exceed traj duration, stop control!");
      geometry_msgs::Twist msg;
      msg.linear.x = 0.0;  // v = 0.0
      msg.angular.z = 0.0;
      vel_cmd_pub_.publish(msg);
      return;
    }
    // set refer states

    int ret = 0;

    std::vector<Eigen::Vector2d> ctrl_points = mpcPtr_->setTrackingPoint(time_from_start);
    visualizeCarCmd(ctrl_points, ros::Time::now());

    Rho = mpcPtr_->getObjectRho();
    Rho2 = mpcPtr_->getObjectRho2();
    
    if (controller_ == 1) {
      ros::Time t1 = ros::Time::now();
      ret = mpcPtr_->solveNMPC(state_);
      ros::Time t2 = ros::Time::now();
      double solve_time = (t2 - t1).toSec();
      // @NOTE solver time should be less than dt=0.05s
      std::cout << "solve nmpc costs: " << 1e3 * solve_time << "ms" << std::endl;   
      //calculate average solve_time
      static int i = 0;
      static double ave_solve_time = 0;
      ave_solve_time = (ave_solve_time * i + solve_time) / (i + 1);
      i++;
      std::cout << "ave_solve_time: " << 1e3 * ave_solve_time << std::endl;
    } else if(controller_ == 2){
      ros::Time t1 = ros::Time::now();
      ret = mpcPtr_->solvePID(state_);
      ros::Time t2 = ros::Time::now();
      double solve_time = (t2 - t1).toSec();
      std::cout << "solve pid costs: " << 1e3 * solve_time << "ms" << std::endl;
      //calculate average solve_time
      static int i = 0;
      static double ave_solve_time = 0;
      ave_solve_time = (ave_solve_time * i + solve_time) / (i + 1);
      i++;
      std::cout << "ave_solve_time: " << 1e3 * ave_solve_time << std::endl;
    } /*else if(controller_ == 3){
      ros::Time t1 = ros::Time::now();
      ret = mpcPtr_->solveQP(state_);
      ros::Time t2 = ros::Time::now();
      double solve_time = (t2 - t1).toSec();
      std::cout << "solve QP costs: " << 1e3 * solve_time << "ms" << std::endl;
      //calculate average solve_time
      static int i = 0;
      static double ave_solve_time = 0;
      ave_solve_time = (ave_solve_time * i + solve_time) / (i + 1);
      i++;
      std::cout << "ave_solve_time: " << 1e3 * ave_solve_time << std::endl;
    } else {
      ROS_WARN("wrong controller type!");
    }*/

    assert(ret == 1);
    
    std::vector<Eigen::Vector2d> pred_points = mpcPtr_->getPredictPoints();
    visualizePredictPath(pred_points, ros::Time::now());

    mpc_car::VectorX x;
    mpc_car::VectorU u;
    mpcPtr_->getPredictXU(0, x, u);

    geometry_msgs::Twist msg;

    Eigen::Vector2d cmd;
    mpcPtr_->getCmd(cmd);
    msg.linear.x = cmd(0);
    msg.angular.z = cmd(1);

    vel_cmd_pub_.publish(msg);


    ros::Time t3 = ros::Time::now();
    mpc_car::VectorX prex;
    mpc_car::VectorU u2;
    mpcPtr_->getPredictXU(0, prex, u2);

    mpc_car::VectorX refx;
    refx = mpcPtr_->setTrackingPoint2(time_from_start);

    mpc_car::VectorX odox= state_;

    
    car_msgs::CarState msg2;
    msg2.header.frame_id = "world";
    msg2.header.stamp = ros::Time::now();


    msg2.error_x = (odox(0)-refx(0))*std::cos(refx(2))+(odox(1)-refx(1))*std::sin(refx(2));
    msg2.error_y = -1*(odox(0)-refx(0))*std::sin(refx(2))+(odox(1)-refx(1))*std::cos(refx(2));
    msg2.error_phi= odox(2)-refx(2);
    msg2.prex = prex(0);
    msg2.prey = prex(1);
    msg2.prephi = prex(2);
    msg2.odox = odox(0);
    msg2.odoy = odox(1);
    msg2.odophi = odox(2);
    msg2.refx = refx(0);
    msg2.refy = refx(1);
    msg2.refphi = refx(2);
    state_pub_.publish(msg2);


    mpcPtr_->visualization();

  }
  return;
}
void odom_call_back(const nav_msgs::Odometry::ConstPtr& msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double sin_y_2 = msg->pose.pose.orientation.z;
  double cos_y_2 = msg->pose.pose.orientation.w;
  double phi = 2 * atan2(sin_y_2, cos_y_2);

  //q = w + xi + yj + zk

  //记录odom
  double ll = mpcPtr_->getObjectll();
  x = x - ll*3/5 * std::cos(phi);
  y = y - ll*3/5 * std::sin(phi);

  odom_x = x, odom_y = y,odom_phi = phi;

  Eigen::Vector2d v(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
  int v_flag;
  Eigen::Vector2d vec_phi(cos(phi), sin(phi));
  if (v.dot(vec_phi) > 0) {
    v_flag = 1;
  } else {
    v_flag = -1;
  }
  state_ << x, y, phi, v_flag * v.norm();
  has_odom_ = true;

  double time_from_start = msg->header.stamp.toSec() - traj_start_time_.toSec();
  if(has_traj_ && time_from_start < traj_duration_)
  {
    // on real car 
    //记录traj
    static int i = 0;
    static double ave_dis = 0;
    Eigen::VectorXd traj_state;
    traj_state.resize(6);
    mpcPtr_->getState(traj_state, time_from_start, 0);
    traj_x = traj_state(0);
    traj_y = traj_state(1);
    traj_phi = traj_state(2);
    car_msgs::Error error;
    error.x = traj_x - odom_x;
    error.y = traj_y - odom_y;
    // error.x = state_(0);
    // error.y = state_(1);
    double dis = sqrt(error.x * error.x + error.y * error.y); 
    error.dis = dis;
    error.phi = traj_phi - odom_phi;
    error.track_x = traj_x;
    error.track_y = traj_y;
    error.track_phi = traj_phi;
    error_pub_.publish(error);
    ave_dis = (ave_dis * i + dis) / (i + 1);
    i++;
    std::cout << "ave_dis: " << ave_dis << std::endl;
  }
}

void traj_call_back(const car_msgs::CarTrajS::ConstPtr& msg) {
  double duration_total = 0;
  std::cout<<msg->size<<std::endl;

  for(int i = 0 ; i< msg->size ;i++){
    if (msg->trajvector[i].order != 5) {
      ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
      return;
    }
    if (msg->trajvector[i].duration.size() * (msg->trajvector[i].order + 1) != msg->trajvector[i].coef_x.size()) {
      ROS_ERROR("[traj_server] WRONG trajectory parameters!");
      return;
    }
    
    for(int j = 0 ;j < msg->trajvector[i].duration.size(); j++){
      duration_total += msg->trajvector[i].duration.at(j);
    }
  }

  traj_duration_ = duration_total;
  mpcPtr_->setRefTraj(msg);
  has_traj_ = true;
  traj_start_time_ = msg->trajvector[0].start_time;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle nh("~");

  mpcPtr_ = std::make_shared<mpc_car::MpcCar>(nh);
  nh.getParam("MPC_dt", dt);
  nh.getParam("MPC_N", n);
  nh.getParam("delay", delay_);
  nh.getParam("controller", controller_);
  plan_timer_ = nh.createTimer(ros::Duration(dt), plan_timer_callback);

  odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 1, odom_call_back);

  traj_sub_ = nh.subscribe<car_msgs::CarTrajS>("car_traj", 1, traj_call_back);
  
  vel_cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/real_cmd", 1);

  state_pub_ = nh.advertise<car_msgs::CarState>("/state_cmd",1);
  cmdPub = nh.advertise<visualization_msgs::Marker>("/visualization/ctrl_traj", 1);
  predPub = nh.advertise<visualization_msgs::Marker>("/visualization/pred_path", 1);
  //发布节点
  error_pub_ = nh.advertise<car_msgs::Error>("/error",1);

  double mpc_duration_ = n * dt;
  if(mpc_duration_ <= 0)
  {
    ROS_ERROR("[Controller]: traj duration %f ?", &mpc_duration_);
  }
  ros::spin();
}
