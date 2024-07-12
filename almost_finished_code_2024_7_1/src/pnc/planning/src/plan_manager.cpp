#include <planning/plan_manager.h>

PlanManager::PlanManager(ros::NodeHandle& nh) : nh_(nh), has_odom_(false){
  vhc_param_.reset(new VehicleParam());
  map_.reset(new mapping::MapManager(nh_));
  kino_path_finder_.reset(new path_searching::KinoAstar(nh_, map_, vhc_param_));
  
  Config config;
  Config::loadParameters(config, nh_);
 
  ploy_traj_opt_.reset(new PolyTrajOptimizer);
  ploy_traj_opt_->setParam(nh_, config);
 
  nh_.getParam("traj_piece_duration", traj_piece_duration_);
  nh_.getParam("traj_res", traj_res);
  nh_.getParam("dense_traj_res", dense_traj_res);
  nh_.getParam("N", N);
  nh_.getParam("dt", dt);
  nh_.getParam("replan_rate", replan_freq);
  nh_.getParam("replan_error_d", replan_error_d_);
  nh_.getParam("replan_error_theta", replan_error_theta_);
    
  // DenseKinopathPub = nh_.advertise<nav_msgs::Path>("/vis_dense_kino_traj", 1);
  Debugtraj1Pub = nh_.advertise<nav_msgs::Path>("/debug/vis_traj_1", 1);
  DebugCorridorPub = nh_.advertise<visualization_msgs::Marker>("/debug/corridor", 1);  
  dense_kino_pub_ = nh_.advertise<nav_msgs::Path>("path", 1);
  ctrl_pub_ = nh_.advertise<car_msgs::CarTrajS>("ctrl", 5);
  traj_path_pub = nh_.advertise<nav_msgs::Path>("/vis_traj", 1);
  wholebody_traj_pub = nh_.advertise<visualization_msgs::Marker>("/vis_wholebody_traj", 1);
  cmd_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("/planning/ctrl_traj", 1);
  cmd_arrow_vis_ = nh_.advertise<visualization_msgs::Marker>("/planning/ctrl_arrow", 1);

  odom_sub_ = nh_.subscribe("odom", 1, &PlanManager::odomCallback, this);
  goal_sub_ = nh_.subscribe("goal", 1, &PlanManager::goalCallback, this);

  //trajServerTimer = nh_.createTimer(ros::Duration(dt), &PlanManager::traj_server_callback, this);  
  replanTimer = nh_.createTimer(ros::Duration(1.0/replan_freq), &PlanManager::replan_callback, this);
  planSuccess = false;
  exec_state_ = INIT;
}

void PlanManager::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  double x = odom_msg->pose.pose.position.x;
  double y = odom_msg->pose.pose.position.y;
  // double phi = tf::getYaw(odom_msg->pose.pose.orientation);
  // double ll = 0.60854;
  // x = x - ll*3/5 * std::cos(phi);
  // y = y - ll*3/5 * std::sin(phi);
  cur_state_(0) = x;
  cur_state_(1) = y;
  cur_state_(2) = tf::getYaw(odom_msg->pose.pose.orientation);
  double vx = odom_msg->twist.twist.linear.x;
  double vy = odom_msg->twist.twist.linear.y;
  Eigen::Vector2d v(vx, vy);
  int v_flag;
  Eigen::Vector2d vec_phi(cos(cur_state_(2)), sin(cur_state_(2)));
  if (v.dot(vec_phi) > 0) {
    v_flag = 1;
  } else {
    v_flag = -1;
  }
  cur_state_(3) = v_flag * v.norm();

  has_odom_ = true;
}


void PlanManager::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg)
{
  if(!map_->Initialized())
  {
    ROS_ERROR("[Planner]: Map not initialized!");
    return;
  }
  if(!has_odom_)
  {
    ROS_ERROR("[Planner]: No odometry received!");
    return;
  }

  double x = goal_msg->pose.position.x;
  double y = goal_msg->pose.position.y;
  // double ll = 0.60854;
  // double phi = tf::getYaw(goal_msg->pose.orientation);
  // x = x - ll*3/5 * std::cos(phi);
  // y = y - ll*3/5 * std::sin(phi);
  goal_state_(0) = x;
  goal_state_(1) = y;
  goal_state_(2) = tf::getYaw(goal_msg->pose.orientation);
  goal_state_(3) = 0.0;    // v = 0.0

  if(exec_state_ == INIT)
  {
    exec_state_ = NEW_GOAL;
  }
  else if(exec_state_ == PLANNING)
  {
    exec_state_ = NEW_GOAL;
  }
  
}

bool PlanManager::plan()
{
  bool planOnceSuccess = true;
  double frontendt1 = ros::Time::now().toSec();
  planOnceSuccess = getKinoPath(goal_state_);
  double frontendt2 = ros::Time::now().toSec();
  // ROS_INFO_STREAM("front_end time is: "<<1000.0*(frontendt2-frontendt1)<<" ms");
  if (!planOnceSuccess){
    ROS_ERROR("[PlanManager] fail to get the front-end.\n");
    return false;   
  }
  else{
    // astar visualization
    nav_msgs::Path vis_pathmsg;
    std::vector<Eigen::Vector4d> trajlist = kino_path_finder_->SamplePosList(200);
    for(auto pos : trajlist){
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "world";
      pose.pose.position.x = pos[0];//x
      pose.pose.position.y = pos[1];//y
      pose.pose.position.z = 0.2;
      pose.header.stamp =  ros::Time().now();//time
      vis_pathmsg.poses.push_back(pose);
    }
    vis_pathmsg.header.frame_id = "world";
    dense_kino_pub_.publish(vis_pathmsg);
  }

  // optimize the trajectories
  double backendt1 = ros::Time::now().toSec();
  planOnceSuccess = RunMINCO();
  double backendt2 = ros::Time::now().toSec();
  if(!planOnceSuccess)
  {
    ROS_ERROR("[PlanManager] fail to optimize the trajectories.\n");
    return false;
  }
  planSuccess = true;
  if(exec_state_ == NEW_GOAL)
  {
    exec_state_ = PLANNING;
  }

  // visualize the trajectory
  std::unique_ptr<plan_utils::SingulTrajData> traj_ptr(new plan_utils::SingulTrajData(traj_container_.singul_traj));
  next_traj_ = std::move(traj_ptr);
  

  return planOnceSuccess;
}

bool PlanManager::CheckReplan(){
      if(exec_state_ == INIT) return false;
      //return 1: replan 0: not
      if(executing_traj_==nullptr) return true;
      bool is_near = false;
      bool is_collision = false;
      bool is_close_turnPoint = false;
      bool large_ctrl_error = false;
      double cur_time = ros::Time::now().toSec();
      Eigen::Vector2d localTarget;
      localTarget = executing_traj_->back().traj.getPos(executing_traj_->back().duration);
      double totaltrajTime = 0.0;
      for(int i = 0; i<executing_traj_->size(); i++){
          totaltrajTime += executing_traj_->at(i).duration;
      }
      // is control error too large?
      double t = ros::Time::now().toSec();
      Eigen::Vector3d exec_state;
      int tmp_exe_index = exe_traj_index_;  // change exe_traj_index_ in traj server callback
      double t_state = t - executing_traj_->at(tmp_exe_index).start_time /*+ 0.2*/;
      executing_traj_->at(tmp_exe_index).traj.GetState3d(t_state, exec_state);
      double e_dis = (cur_state_.head(2) - exec_state.head(2)).norm();
      if(e_dis > replan_error_d_)
      {
        ROS_WARN("too large control distance error, replan");
        std::cout << "exec_state: " << exec_state.transpose() << "cur_state_: " << cur_state_.transpose() << std::endl;
        return true;
      }
      double e_angle = std::fabs(exec_state(2) - cur_state_(2));
      e_angle = e_angle > 2*M_PI ? e_angle - 2 * M_PI : e_angle;
      e_angle = std::min(e_angle, 2*M_PI - e_angle);   // angle filtering
      if(e_angle > replan_error_theta_)
      {
        ROS_WARN("too large control angle error, replan");
        std::cout << "exec_state: " << exec_state.transpose() << "cur_state_: " << cur_state_.transpose() << std::endl;
        return true;
      } 

      //is close to turnPoint?
      if(exe_traj_index_ == final_traj_index_) is_close_turnPoint = false;  //final_traj_index_
      else{
        if((executing_traj_->at(exe_traj_index_).end_time - cur_time)<2.5)
          is_close_turnPoint = true;
      }
      //is near?
      if((executing_traj_->back().end_time - cur_time)<2*totaltrajTime / 3.0) is_near = true;
      else is_near = false;
      if(is_near && !is_close_turnPoint&&(localTarget-goal_state_.head(2)).norm()>0.1){
        return true;
      }
      //collision-check
      for(int i = 0; i < executing_traj_->size(); i++){
        for(double t = 0.0; t < executing_traj_->at(i).duration; t+=0.05){
          Eigen::Vector2d pos;
          Eigen::Vector3d state;
          double yaw;
          pos = executing_traj_->at(i).traj.getPos(t);
          yaw = executing_traj_->at(i).traj.getAngle(t);
          state << pos[0],pos[1],yaw;
          is_collision = map_->CheckIfCollisionUsingPosAndYaw(state);
          if(is_collision)  return true;   
        }

      }
      // executing_traj_/
      
      return false;
  }

void PlanManager::replan_callback(const ros::TimerEvent& event)
{
  static int plan_time = 0;
  double current_time = ros::Time::now().toSec();
  if (executing_traj_!=nullptr && current_time > executing_traj_->at(final_traj_index_).end_time) {

      printf("[PlanManager]: Mission complete.\n");
      m.lock();
      executing_traj_.release();
      m.unlock();
      exec_state_ = INIT;
      return;
  }
  if(CheckReplan()){
    plan_time++;
    std::cout << "\033[032m ******************* REPLAN " << plan_time << " ******************* \033[0m"<< std::endl;;
    bool plan_success = plan();
    if(plan_success){
      m.lock();
      executing_traj_ = std::move(next_traj_);
      next_traj_ = nullptr;
      exe_traj_index_ = 0;
      final_traj_index_ = executing_traj_->size()-1;
      m.unlock();
      //@zy
      //pub here
      pub_traj();
      displayPolyTraj(executing_traj_);
    }
  }
  return;
}

//@zy
//send multiple trajvector
void PlanManager::pub_traj() {
    car_msgs::CarTrajS trajvector_msgS;
    if(executing_traj_ == nullptr)
      return;
    std::cout<<executing_traj_->size()<<std::endl;
    trajvector_msgS.size = executing_traj_->size();

    for(int i = 0 ;i < kino_trajs_.size(); i++)
      trajvector_msgS.singul.push_back(kino_trajs_[i].singul);

    for(int i = 0 ;i < executing_traj_->size(); i++){
      if(executing_traj_->at(i).duration < 1e-5)
        continue;
      car_msgs::CarTraj trajvector_msg;
      int thistraj_id = i;
      trajvector_msg.traj_id = thistraj_id;
      trajvector_msg.order = 5;
      Eigen::VectorXd durs = executing_traj_-> at(i).traj.getDurations();
      int piece_num = executing_traj_-> at(i).traj.getPieceNum();

      trajvector_msg.duration.resize(piece_num);
      trajvector_msg.coef_x.resize(6 * piece_num);
      trajvector_msg.coef_y.resize(6 * piece_num);

      for (int j = 0; j < piece_num; ++j){
        trajvector_msg.duration[j] = durs(j);
        plan_utils::CoefficientMat cMat = executing_traj_-> at(i).traj[j].getCoeffMat();
        int j6 = j * 6;
        for (int k = 0; k < 6; k++) {
          trajvector_msg.coef_x[j6 + k] = cMat(0, k);
          trajvector_msg.coef_y[j6 + k] = cMat(1, k);
        }
      }

      ros::Time tmptime(executing_traj_-> at(i).start_time);
      trajvector_msg.start_time = tmptime;
      trajvector_msgS.trajvector.push_back(trajvector_msg);
    }
    ctrl_pub_.publish(trajvector_msgS);
}

//@zy
//unused function
void PlanManager::traj_server_callback(const ros::TimerEvent& event)
{
    m.lock();
    ros::Time curTime = ros::Time::now();
    double t = curTime.toSec();
    if (executing_traj_ == nullptr ||exe_traj_index_ > final_traj_index_ ||
     executing_traj_->at(exe_traj_index_).duration < 1e-5)
    {
      // DO NOT publish cmd
      m.unlock();
      return;
    }

    if(executing_traj_->at(exe_traj_index_).end_time <= t) {
      exe_traj_index_++;
    }
    if(exe_traj_index_ > final_traj_index_){
      ROS_INFO("[traj_generator] trajectory finished!");
      m.unlock();
      return;
    }

    int tmp_exe_index = exe_traj_index_;
    // car_msgs::CarTraj cmd;
    // cmd.header.stamp = curTime;
    // cmd.n = N;
    // cmd.dt = dt;
    for(int i = 0; i < N; i++)
    {
      Eigen::VectorXd state; // x y yaw cur v
      state.resize(5);
      double t_state;
      double t_cmd = i * dt + t;
      if(t_cmd < executing_traj_->at(tmp_exe_index).end_time)
      {
        t_state = t_cmd - executing_traj_->at(tmp_exe_index).start_time;
        executing_traj_->at(tmp_exe_index).traj.GetState(t_state, state);
      }
      else if(t_cmd >= executing_traj_->at(tmp_exe_index).end_time && tmp_exe_index < executing_traj_->size() - 1)
      {
        tmp_exe_index++;   // bug here
        t_state = t_cmd - executing_traj_->at(tmp_exe_index).start_time;
        executing_traj_->at(tmp_exe_index).traj.GetState(t_state, state);
      }
      else // reach the end
      {
        t_state = executing_traj_->at(tmp_exe_index).duration;
        executing_traj_->at(tmp_exe_index).traj.GetState(t_state, state);
      }
      // cmd.x.push_back(state(0));
      // cmd.y.push_back(state(1));
      // cmd.phi.push_back(state(2));
    }
    // ctrl_pub_.publish(cmd);
    // std::cout << "cmd yaw: " << cmd.phi[0] << std::endl;

    // visualizeCarCmd(cmd, curTime);
    m.unlock();
    return;

}

// void PlanManager::visualizeCarCmd(const car_msgs::CarTraj &msg, ros::Time timeStamp)
// {
//     visualization_msgs::Marker Marker;
//     Marker.id = 0;
//     Marker.type = visualization_msgs::Marker::SPHERE_LIST;
//     Marker.header.stamp = timeStamp;
//     Marker.header.frame_id = "world";
//     Marker.pose.orientation.w = 1.00;
//     Marker.action = visualization_msgs::Marker::ADD;
//     Marker.ns = "CarCmd";
//     Marker.color.r = 1.00;
//     Marker.color.g = 0.00;
//     Marker.color.b = 0.00;
//     Marker.color.a = 1.00;
//     Marker.scale.x = 0.20;
//     Marker.scale.y = 0.20;
//     Marker.scale.z = 0.20;

//     for (u_int i = 0; i < msg.x.size(); i++)
//     {
//         geometry_msgs::Point point;
//         point.x = msg.x[i];
//         point.y = msg.y[i];
//         point.z = 0.0;
//         Marker.points.push_back(point);
//     }
//     cmd_vis_pub_.publish(Marker);

//     // visualize arrow
//     visualization_msgs::Marker Arrow;
//     Arrow.id = 0;
//     Arrow.type = visualization_msgs::Marker::ARROW;
//     Arrow.header.stamp = timeStamp;
//     Arrow.header.frame_id = "world";
//     Arrow.pose.orientation.w = 1.00;
//     Arrow.action = visualization_msgs::Marker::ADD;
//     Arrow.ns = "CarCmd";
//     Arrow.color.r = 0.00;
//     Arrow.color.g = 1.00;
//     Arrow.color.b = 0.00;
//     Arrow.color.a = 1.00;
//     Arrow.scale.x = 0.05;
//     Arrow.scale.y = 0.15;
//     Arrow.scale.z = 0.00;

//     double length = 0.5;
//     geometry_msgs::Point start, end;
//     start.x = msg.x[0];
//     start.y = msg.y[0];
//     start.z = 0.0;
//     end.x = msg.x[0] + length * cos(msg.phi[0]);
//     end.y = msg.y[0] + length * sin(msg.phi[0]);
//     end.z = 0.0;
//     Arrow.points.push_back(start);
//     Arrow.points.push_back(end);
//     cmd_arrow_vis_.publish(Arrow);
// }

bool PlanManager::getKinoPath(Eigen::Vector4d &end_state)
{
    Eigen::Vector2d init_ctrl(0.0, 0.0);  

    kino_path_finder_->reset();
    double searcht1 = ros::Time::now().toSec();

    // std::cout<<"start state: "<<start_state.transpose()<<" end_state: "<<end_state.transpose()<<std::endl;
    // std::cout<<"init ctrl: "<<init_ctrl.transpose()<<std::endl;
    int status = kino_path_finder_->search(cur_state_, init_ctrl, end_state, true);
    double searcht2 = ros::Time::now().toSec();
    // std::cout<<"search time: "<<(searcht2-searcht1)<<std::endl;
    if (status == path_searching::KinoAstar::NO_PATH)
    {
      std::cout << "[kino replan]: kinodynamic search fail!" << std::endl;

      // retry searching with discontinuous initial state
      kino_path_finder_->reset();
      status = kino_path_finder_->search(cur_state_, init_ctrl, end_state, false);
      if (status == path_searching::KinoAstar::NO_PATH)
      {
        std::cout << "[kino replan]: Can't find path." << std::endl;
        return false;
      }
      else
      {
        std::cout << "[kino replan]: retry search success." << std::endl;
      }
    }
    else
    {
      // std::cout << "[kino replan]: kinodynamic search success." << std::endl;
    }

    kino_path_finder_->getKinoNode(kino_trajs_);
    
    // ROS_WARN("hzc debug kinodynamic search");
    // std::cout << " kino_trajs_.size() :" <<   kino_trajs_.size()  << std::endl;
    // std::cout << " kino_trajs_.at(0).start_state  :" <<  kino_trajs_.at(0).start_state << std::endl;
    // std::cout << " kino_trajs_.at(0).final_state :" <<   kino_trajs_.at(0).final_state << std::endl;
    //ros::shutdown();
    return true;
}

bool PlanManager::RunMINCO(){
    
    traj_container_.clearSingul();
    Eigen::MatrixXd flat_finalState(2, 3),  flat_headState(2,3);  // no use variables
    Eigen::VectorXd ego_piece_dur_vec;
    Eigen::MatrixXd ego_innerPs;
    // ROS_WARN("begin to run minco");
    nav_msgs::Path debug_msg0,debug_msg1;
    display_hPolys_.clear();
    double worldtime =  ros::Time::now().toSec();  // it should be fine
    double basetime = 0.0;

    /*try to merge optimization process*/
    std::vector<std::vector<Eigen::MatrixXd>> sfc_container;
    std::vector<int> singul_container;
    Eigen::VectorXd duration_container;
    std::vector<Eigen::MatrixXd> waypoints_container;
    std::vector<Eigen::MatrixXd> iniState_container,finState_container;
    duration_container.resize(kino_trajs_.size());

    for(unsigned int i = 0; i < kino_trajs_.size(); i++){
      double timePerPiece = traj_piece_duration_;
      plan_utils::FlatTrajData kino_traj = kino_trajs_.at(i);
      singul_container.push_back(kino_traj.singul);
      std::vector<Eigen::Vector3d> pts = kino_traj.traj_pts;
      plan_utils::MinJerkOpt initMJO;
      plan_utils::Trajectory initTraj;
      int piece_nums;
      double initTotalduration = 0.0;
      for(const auto pt : pts){
        initTotalduration += pt[2];
      }
      piece_nums = std::max(int(initTotalduration / timePerPiece + 0.5),2);
      timePerPiece = initTotalduration / piece_nums; 
      ego_piece_dur_vec.resize(piece_nums);
      ego_piece_dur_vec.setConstant(timePerPiece);
      duration_container[i] = timePerPiece * piece_nums;
      ego_innerPs.resize(2, piece_nums-1);
      std::vector<Eigen::Vector3d> statelist;
      double res_time = 0;
      for(int i = 0; i < piece_nums; i++ ){
        int resolution;
        if(i==0||i==piece_nums-1){
          resolution = dense_traj_res;
        }
        else{
          resolution = traj_res;
        }
        for(int k = 0; k <= resolution; k++){
          double t = basetime+res_time + 1.0*k/resolution*ego_piece_dur_vec[i];
          Eigen::Vector3d pos = kino_path_finder_->evaluatePos(t);
          statelist.push_back(pos);
          if(k==resolution && i!=piece_nums-1){
            ego_innerPs.col(i) = pos.head(2); 
          }
        } 
        res_time += ego_piece_dur_vec[i];
      }
      // std::cout<<"s: "<<kino_traj.singul<<"\n";
      getRectangleConst(statelist);
      sfc_container.push_back(hPolys_);
      display_hPolys_.insert(display_hPolys_.end(),hPolys_.begin(),hPolys_.end());
      waypoints_container.push_back(ego_innerPs);
      iniState_container.push_back(kino_traj.start_state);
      finState_container.push_back(kino_traj.final_state);
      basetime += initTotalduration;
      //visualization
      initMJO.reset(ego_piece_dur_vec.size());
      initMJO.generate(ego_innerPs, timePerPiece,kino_traj.start_state, kino_traj.final_state);
      initTraj = initMJO.getTraj(kino_traj.singul);
      {
        for(double t  = 0.0; t <= initTraj.getTotalDuration(); t+=0.01){
          geometry_msgs::PoseStamped pose;
          pose.header.frame_id = "world";
          pose.pose.position.x = initTraj.getPos(t)[0];//x
          pose.pose.position.y = initTraj.getPos(t)[1];//y
          pose.pose.position.z = 0.2;
          debug_msg1.poses.push_back(pose);
        }
        debug_msg1.header.frame_id = "world";
      }
      Debugtraj1Pub.publish(debug_msg1);  
    }
    Debugtraj1Pub.publish(debug_msg1);

    // std::cout<<"try to optimize!\n";
    
    int flag_success = ploy_traj_opt_->OptimizeTrajectory(iniState_container, finState_container, 
                                                        waypoints_container,duration_container, 
                                                        sfc_container,  singul_container,worldtime,0.0);
    // std::cout<<"optimize ended!\n";
   


    if (flag_success)
    {
        // std::cout << "[PolyTrajManager] Planning success ! " << std::endl;
        for(unsigned int i = 0; i < kino_trajs_.size(); i++){
          traj_container_.addSingulTraj( (*ploy_traj_opt_->getMinJerkOptPtr())[i].getTraj(singul_container[i]), worldtime, 0); // todo time // @zhiwei set ego_id to 0 
          // std::cout<<"init duration: "<<duration_container[i]<<std::endl;
          // std::cout<<"pieceNum: " << waypoints_container[i].cols() + 1 <<std::endl;
          // std::cout<<"optimized total duration: "<<(*ploy_traj_opt_->getMinJerkOptPtr())[i].getTraj(1).getTotalDuration()<<std::endl;
          // std::cout<<"optimized jerk cost: "<<(*ploy_traj_opt_->getMinJerkOptPtr())[i].getTrajJerkCost()<<std::endl;
          worldtime = traj_container_.singul_traj.back().end_time;
        }

        //debug
        double max_acc = -1, max_lat  = -1;
        for(unsigned int i = 0; i < kino_trajs_.size(); i++){
          plan_utils::Trajectory  traj = traj_container_.singul_traj[i].traj;
          for(double rest = 0.0; rest <= traj.getTotalDuration(); rest += 0.01){
            double lonacc = std::fabs(traj.getAcc(rest));
            double latacc = std::fabs(traj.getLatAcc(rest));
            max_acc = max(max_acc,lonacc);
            max_lat = max(max_lat,latacc);

          }
          int sing = kino_trajs_[i].singul;
          if(sing == 1)
            std::cout <<"\033[35mforward trajid: "<<i<<" duration: "<< traj.getTotalDuration()<< "\033[0m"<<std::endl;
          else if(sing==-1)
            std::cout <<"\033[36mbackward trajid: "<<i<<" duration: "<< traj.getTotalDuration()<< "\033[0m"<<std::endl;
        }

        // ROS_INFO_STREAM("max lon acc: " << max_acc <<" max lat acc: "<<max_lat);
    }
    else{
        ROS_ERROR("[PolyTrajManager] Planning fails! ");
        return false;
    }

    // @zhiwei dont know if this is necessary
    // if(is_init){
    //   //reset the timeStamp
    //   for(auto & it:traj_container_.singul_traj ){
    //     it.start_time = ros::Time::now().toSec()-head_state_.time_stamp+it.start_time;
    //     it.end_time = ros::Time::now().toSec()-head_state_.time_stamp+it.end_time;
    //   }
    // }

    return flag_success;

}

void PlanManager::getRectangleConst(std::vector<Eigen::Vector3d> statelist){
    hPolys_.clear();
    double resolution = map_->GetMapResolution();
    double step = resolution * 1.0;
    double limitBound = 10.0;
    visualization_msgs::Marker  carMarkers;
    //generate a rectangle for this state px py yaw
    for(const auto state : statelist){
      //generate a hPoly
      Eigen::MatrixXd hPoly;
      hPoly.resize(4, 4);
      Eigen::Matrix<int,4,1> NotFinishTable = Eigen::Matrix<int,4,1>(1,1,1,1);      
      Eigen::Vector2d sourcePt = state.head(2);
      Eigen::Vector2d rawPt = sourcePt;
      double yaw = state[2];
      VehicleParam sourceVp,rawVp;
      Eigen::Matrix2d egoR;
      egoR << cos(yaw), -sin(yaw),
              sin(yaw), cos(yaw);
      VehicleParam vptest;

      bool test = map_->CheckIfCollisionUsingPosAndYaw(state);

      if(test){
        ROS_WARN(
          "init traj is not safe?"
        );
        // std::cout<<"yaw: "<<yaw<<"\n";
        carMarkers.action = visualization_msgs::Marker::ADD;
        carMarkers.id = 0;
        carMarkers.type = visualization_msgs::Marker::LINE_LIST;
        carMarkers.pose.orientation.w = 1.00;
        carMarkers.ns = "libaicorridorF";
        carMarkers.color.r = 0.00;
        carMarkers.color.g = 0.00;
        carMarkers.color.b = 0.00;
        carMarkers.color.a = 1.00;
        carMarkers.scale.x = 0.05;
        carMarkers.header.frame_id = "world";
        geometry_msgs::Point point1;
        geometry_msgs::Point point2;
        geometry_msgs::Point point3;
        geometry_msgs::Point point4;
        Eigen::Matrix2d R;
        R << cos(yaw),-sin(yaw),
            sin(yaw),cos(yaw);
        Eigen::Vector2d offset1, tmp1;
        offset1 = R*Eigen::Vector2d(vptest.length()/2.0+vptest.d_cr(),vptest.width()/2.0);
        tmp1 = state.head(2)+offset1;
        point1.x = tmp1[0]; 
        point1.y = tmp1[1];
        point1.z = 0;
        Eigen::Vector2d offset2, tmp2;
        offset2 = R*Eigen::Vector2d(vptest.length()/2.0+vptest.d_cr(),-vptest.width()/2.0);
        tmp2 = state.head(2)+offset2;
        point2.x = tmp2[0]; 
        point2.y = tmp2[1];
        point2.z = 0;
        Eigen::Vector2d offset3, tmp3;
        offset3 = R*Eigen::Vector2d(-vptest.length()/2.0+vptest.d_cr(),-vptest.width()/2.0);
        tmp3 = state.head(2)+offset3;
        point3.x = tmp3[0]; 
        point3.y = tmp3[1];
        point3.z = 0;
        Eigen::Vector2d offset4, tmp4;
        offset4 = R*Eigen::Vector2d(-vptest.length()/2.0+vptest.d_cr(),vptest.width()/2.0);
        tmp4 = state.head(2)+offset4;
        point4.x = tmp4[0]; 
        point4.y = tmp4[1];
        point4.z = 0;
        carMarkers.points.push_back(point1);
        carMarkers.points.push_back(point2);
        carMarkers.points.push_back(point2);
        carMarkers.points.push_back(point3);
        carMarkers.points.push_back(point3);
        carMarkers.points.push_back(point4);
        carMarkers.points.push_back(point4);
        carMarkers.points.push_back(point1);

      }
       

      Eigen::Vector4d expandLength;
      expandLength << 0.0, 0.0, 0.0, 0.0;
      //dcr width length
      while(NotFinishTable.norm()>0){ 
        //+dy  +dx -dy -dx  
        for(int i = 0; i<4; i++){
            if(!NotFinishTable[i]) continue;
            //get the new source and vp
            Eigen::Vector2d NewsourcePt = sourcePt;
            VehicleParam NewsourceVp = sourceVp;
            Eigen::Vector2d point1,point2,newpoint1,newpoint2;

            bool isocc = false;
            switch (i)
            {
            //+dy
            case 0:
              point1 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0);
              point2 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0);
              newpoint1 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0+step);   
              newpoint2 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0+step);
              //1 new1 new1 new2 new2 2
              isocc = map_->CheckIfCollisionUsingLine(point1,newpoint1,resolution/2.0);   // TODO TODO TODO
              if(isocc){
                NotFinishTable[i] = 0.0;
                break;
              }
              isocc = map_->CheckIfCollisionUsingLine(newpoint1,newpoint2,resolution/2.0);
              if(isocc){
                NotFinishTable[i] = 0.0;
                break;
              }
              isocc = map_->CheckIfCollisionUsingLine(newpoint2,point2,resolution/2.0);
              if(isocc){
                NotFinishTable[i] = 0.0;
                break;
              }
              expandLength[i] += step;
              if(expandLength[i] >= limitBound){
                NotFinishTable[i] = 0.0;
                break;
              }
              NewsourcePt = NewsourcePt + egoR * Eigen::Vector2d(0,step/2.0);
              NewsourceVp.set_width(NewsourceVp.width() + step);
              sourcePt = NewsourcePt;
              sourceVp = NewsourceVp;
              break;
            //+dx
            case 1:
              point1 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
              point2 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0);
              newpoint1 = sourcePt + egoR * Eigen::Vector2d(step+sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);   
              newpoint2 = sourcePt + egoR * Eigen::Vector2d(step+sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0);
              //1 new1 new1 new2 new2 2
              isocc = map_->CheckIfCollisionUsingLine(point1,newpoint1,resolution/2.0);
              if(isocc){
                NotFinishTable[i] = 0.0;
                break;
              }
              isocc = map_->CheckIfCollisionUsingLine(newpoint1,newpoint2,resolution/2.0);
              if(isocc){
                NotFinishTable[i] = 0.0;
                break;
              }
              isocc = map_->CheckIfCollisionUsingLine(newpoint2,point2,resolution/2.0);
              if(isocc){
                NotFinishTable[i] = 0.0;
                break;
              }
              expandLength[i] += step;
              if(expandLength[i] >= limitBound){
                NotFinishTable[i] = 0.0;
                break;
              }
              NewsourcePt = NewsourcePt + egoR * Eigen::Vector2d(step/2.0,0.0);
              NewsourceVp.set_length(NewsourceVp.length() + step);
              sourcePt = NewsourcePt;
              sourceVp = NewsourceVp;
              break;
            //-dy
            case 2:
              point1 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
              point2 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
              newpoint1 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0-step);   
              newpoint2 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0-step);
              //1 new1 new1 new2 new2 2
              isocc = map_->CheckIfCollisionUsingLine(point1,newpoint1,resolution/2.0);
              if(isocc){
                NotFinishTable[i] = 0.0;
                break;
              }
              isocc = map_->CheckIfCollisionUsingLine(newpoint1,newpoint2,resolution/2.0);
              if(isocc){
                NotFinishTable[i] = 0.0;
                break;
              }
              isocc = map_->CheckIfCollisionUsingLine(newpoint2,point2,resolution/2.0);
              if(isocc){
                NotFinishTable[i] = 0.0;
                break;
              }
              expandLength[i] += step;
              if(expandLength[i] >= limitBound){
                NotFinishTable[i] = 0.0;
                break;
              }
              NewsourcePt = NewsourcePt + egoR * Eigen::Vector2d(0,-step/2.0);
              NewsourceVp.set_width(NewsourceVp.width() + step);
              sourcePt = NewsourcePt;
              sourceVp = NewsourceVp;
              break;
            //-dx
            case 3:
              point1 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0);
              point2 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
              newpoint1 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr()-step,sourceVp.width()/2.0);
              newpoint2 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr()-step,-sourceVp.width()/2.0);
              //1 new1 new1 new2 new2 2
              isocc = map_->CheckIfCollisionUsingLine(point1,newpoint1,resolution/2.0);
              if(isocc){
                NotFinishTable[i] = 0.0;
                break;
              }
              isocc = map_->CheckIfCollisionUsingLine(newpoint1,newpoint2,resolution/2.0);
              if(isocc){
                NotFinishTable[i] = 0.0;
                break;
              }
              isocc = map_->CheckIfCollisionUsingLine(newpoint2,point2,resolution/2.0);
              if(isocc){
                NotFinishTable[i] = 0.0;
                break;
              }
              expandLength[i] += step;
              if(expandLength[i] >= limitBound){
                NotFinishTable[i] = 0.0;
                break;
              }
              NewsourcePt = NewsourcePt + egoR * Eigen::Vector2d(-step/2.0,0.0);
              NewsourceVp.set_length(NewsourceVp.length() + step);
              sourcePt = NewsourcePt;
              sourceVp = NewsourceVp;
              break;
            }   
        }
     }
      Eigen::Vector2d point1,norm1;
      point1 = rawPt+egoR*Eigen::Vector2d(rawVp.length()/2.0+rawVp.d_cr()+expandLength[1],rawVp.width()/2.0+expandLength[0]);
      norm1 << -sin(yaw), cos(yaw);
      hPoly.col(0).head<2>() = norm1;
      hPoly.col(0).tail<2>() = point1;
      Eigen::Vector2d point2,norm2;
      // point2 = sourcePt+egoR*Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
      point2 = rawPt+egoR*Eigen::Vector2d(rawVp.length()/2.0+rawVp.d_cr()+expandLength[1],-rawVp.width()/2.0-expandLength[2]);
      norm2 << cos(yaw), sin(yaw);
      hPoly.col(1).head<2>() = norm2;
      hPoly.col(1).tail<2>() = point2;
      Eigen::Vector2d point3,norm3;
      // point3 = sourcePt+egoR*Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
      point3 = rawPt+egoR*Eigen::Vector2d(-rawVp.length()/2.0+rawVp.d_cr()-expandLength[3],-rawVp.width()/2.0-expandLength[2]);
      norm3 << sin(yaw), -cos(yaw);
      hPoly.col(2).head<2>() = norm3;
      hPoly.col(2).tail<2>() = point3;
      Eigen::Vector2d point4,norm4;
      // point4 = sourcePt+egoR*Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0);
      point4 = rawPt+egoR*Eigen::Vector2d(-rawVp.length()/2.0+rawVp.d_cr()-expandLength[3],rawVp.width()/2.0+expandLength[0]);
      norm4 << -cos(yaw), -sin(yaw);
      hPoly.col(3).head<2>() = norm4;
      hPoly.col(3).tail<2>() = point4;
      hPolys_.push_back(hPoly);
  };
  DebugCorridorPub.publish(carMarkers);
  return;
}

void PlanManager::displayPolyTraj(std::unique_ptr<plan_utils::SingulTrajData> &display_traj){


  nav_msgs::Path path_msg;
  geometry_msgs::PoseStamped pose;
  pose.pose.orientation.w = 1.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  for (unsigned int i = 0; i < display_traj->size(); ++i){

    double total_duration = display_traj->at(i).duration;
    for (double t = 0; t <= total_duration; t += 0.01){

      Eigen::Vector2d pt = display_traj->at(i).traj.getPos(t);
      pose.pose.position.x = pt(0);
      pose.pose.position.y = pt(1);
      pose.pose.position.z = 0.2;
      path_msg.poses.push_back(pose);
    }

  }

  path_msg.header.frame_id = "world";
  traj_path_pub.publish(path_msg);

  double last_debugyaw =  display_traj->at(0).traj.getAngle(0.0);


  for (unsigned int i = 0; i < display_traj->size(); ++i){
    double total_duration = display_traj->at(i).duration;
    for (double t = 0; t <= total_duration; t += 0.01){

      Eigen::Vector2d pt = display_traj->at(i).traj.getPos(t);
      pose.pose.position.x = pt(0);
      pose.pose.position.y = pt(1);
      pose.pose.position.z = 0.2;
      path_msg.poses.push_back(pose);
      Eigen::Vector2d vel = display_traj->at(i).traj.getdSigma(t);
      double yaw = display_traj->at(i).traj.getAngle(t);
      // std::cout<<"pos: "<<pt.transpose()<<" vel: "<<vel.transpose()<<" yaw: "<<yaw<<std::endl;

      // if(fabs(yaw-last_debugyaw)>0.2){
      // }
      last_debugyaw = yaw;
    }

  }
  visualization_msgs::Marker carMarkers;
  carMarkers.header.frame_id = "world";
  carMarkers.header.stamp = ros::Time::now();
  carMarkers.type = visualization_msgs::Marker::LINE_LIST;
  carMarkers.action = visualization_msgs::Marker::DELETE;
  wholebody_traj_pub.publish(carMarkers);
  carMarkers.action = visualization_msgs::Marker::ADD;
  carMarkers.id = 21;
  carMarkers.pose.orientation.w = 1.00;
  carMarkers.ns = "trajwholepub";
  carMarkers.color.r = 1.00;
  carMarkers.color.g = 0.00;
  carMarkers.color.b = 1.00;
  carMarkers.color.a = 1.00;
  carMarkers.scale.x = 0.05;
  geometry_msgs::Point pt;
  for (unsigned int i = 0; i < display_traj->size(); ++i){
    double total_duration = display_traj->at(i).duration;
    for (double t = 0; t <= total_duration; t += 0.1){
      Eigen::Vector2d pos = display_traj->at(i).traj.getPos(t);
      double yaw = display_traj->at(i).traj.getAngle(t);
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = 0.1;
      geometry_msgs::Point point1;
      geometry_msgs::Point point2;
      geometry_msgs::Point point3;
      geometry_msgs::Point point4;
      Eigen::Matrix2d R;
      R << cos(yaw),-sin(yaw),
            sin(yaw),cos(yaw);
      Eigen::Vector2d offset1, tmp1;
      offset1 = R*Eigen::Vector2d(vhc_param_->length()/2.0+vhc_param_->d_cr(),vhc_param_->width()/2.0);
      tmp1 = pos+offset1;
      point1.x = tmp1[0]; 
      point1.y = tmp1[1];
      point1.z = 0;

      Eigen::Vector2d offset2, tmp2;
      offset2 = R*Eigen::Vector2d(vhc_param_->length()/2.0+vhc_param_->d_cr(),-vhc_param_->width()/2.0);
      tmp2 = pos+offset2;
      point2.x = tmp2[0]; 
      point2.y = tmp2[1];
      point2.z = 0;

      Eigen::Vector2d offset3, tmp3;
      offset3 = R*Eigen::Vector2d(-vhc_param_->length()/2.0+vhc_param_->d_cr(),-vhc_param_->width()/2.0);
      tmp3 = pos+offset3;
      point3.x = tmp3[0]; 
      point3.y = tmp3[1];
      point3.z = 0;

      Eigen::Vector2d offset4, tmp4;
      offset4 = R*Eigen::Vector2d(-vhc_param_->length()/2.0+vhc_param_->d_cr(),vhc_param_->width()/2.0);
      tmp4 = pos+offset4;
      point4.x = tmp4[0]; 
      point4.y = tmp4[1];
      point4.z = 0;

      carMarkers.points.push_back(point1);
      carMarkers.points.push_back(point2);

      carMarkers.points.push_back(point2);
      carMarkers.points.push_back(point3);

      carMarkers.points.push_back(point3);
      carMarkers.points.push_back(point4);

      carMarkers.points.push_back(point4);
      carMarkers.points.push_back(point1);

    }

  }

  wholebody_traj_pub.publish(carMarkers);
}