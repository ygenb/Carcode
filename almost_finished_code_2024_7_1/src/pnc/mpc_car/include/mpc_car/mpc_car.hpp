#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <car_msgs/CarTraj.h>
#include <car_msgs/CarTrajS.h> 
#include <car_msgs/CarState.h>
#include <car_msgs/Error.h>
#include <Eigen/Core>
#include <ros/ros.h>
#include <plan_utils/poly_traj_utils.hpp>
#include <iosqp/iosqp.hpp>

#include <deque>

#include "lbfgs_raw.hpp"

double Rho;
double Rho2;

namespace mpc_car {

static constexpr int n = 4;  // state x y phi v
static constexpr int m = 2;  // input a delta
typedef Eigen::Matrix<double, n, n> MatrixA;
typedef Eigen::Matrix<double, n, m> MatrixB;
typedef Eigen::Vector4d VectorG;
typedef Eigen::Vector4d VectorX;
typedef Eigen::Vector2d VectorU;

class MpcCar {
 private:
  ros::NodeHandle nh_;
  ros::Publisher ref_pub_, traj_pub_, traj_delay_pub_;
  bool init_ = false;

  double ll_;
  double dt_;
  double rho_;
  double rho2_;
  int N_;
  double rhoN_;

  double v_max_, a_max_, delta_max_, ddelta_max_;
  double delay_;
  double delay_steer_;

  double kp_v_, ki_v_, kd_v_;
  double kp_phi_, ki_phi_, kd_phi_;

  std::vector<VectorX> predictState_;
  std::vector<Eigen::VectorXd> reference_states_, trajectory_state_;
  std::vector<VectorU> predictInput_;
  std::deque<VectorU> historyInput_;
  int history_length_;
  VectorX x0_observe_;

  std::vector<plan_utils::Trajectory> traj_;
  std::vector<int> traj_singul_;
  std::vector<int> reference_states_singul_;


  // osqp::IOSQP qpSolver_;

  MatrixA Ad_;
  MatrixB Bd_;
  VectorG gd_;
  // x_{k+1} = Ad * x_{k} + Bd * u_k + gd
  Eigen::SparseMatrix<double> P_, q_, A_, l_, u_;
  // Eigen::SparseMatrix<double> P0_, q0_;
  Eigen::SparseMatrix<double> Cx_, lx_, ux_;  // p, v constrains
  Eigen::SparseMatrix<double> Cu_, lu_, uu_;  // a delta vs constrains
  Eigen::SparseMatrix<double> Qx_;


  inline VectorX diff(const VectorX& state,
                      const VectorU& input) const {
    VectorX ds;
    double phi = state(2);
    double v = state(3);
    double a = input(0);
    double delta = input(1);
    ds(0) = v * cos(phi);
    ds(1) = v * sin(phi);
    ds(2) = v / ll_ * tan(delta);
    ds(3) = a;
    return ds;
  }

  inline void step(VectorX& state, const VectorU& input, const double dt) const {
    // Runge–Kutta
    VectorX k1 = diff(state, input);
    VectorX k2 = diff(state + k1 * dt / 2, input);
    VectorX k3 = diff(state + k2 * dt / 2, input);
    VectorX k4 = diff(state + k3 * dt, input);
    state = state + (k1 + k2 * 2 + k3 * 2 + k4) * dt / 6;
  }

  // compute the state according to history input
  VectorX compensateDelay(const VectorX& x0) {
    //mcj and zy changed
    VectorX x0_delay = x0;
    double dt = 0.001;
    int num = ceil((delay_ - delay_steer_) / dt_ );
    for (double t = delay_steer_; t > 0; t -= dt) {
      int i = std::ceil(t / dt_);
      VectorU input1 = historyInput_[history_length_ - num - i]; //+1?
      VectorU input2 = historyInput_[history_length_ - i];
      VectorU input;
      input(0)=input1(0);
      input(1)=input2(1);
      step(x0_delay, input, dt);
    }
    //mcj and zy changed end

    // for (double t = delay_steer_; t > 0; t -= dt) {
    //   int i = std::ceil(t / dt_);
    //   VectorU input = historyInput_[history_length_ - i];
    //   step(x0_steer_delay, input, dt);
    // }

    return x0_delay;
  }
  //zy changed
  VectorX compensateDelay_(const VectorX& x0) {
    VectorX x0_delay = x0;
    // TODO: compensate delay
    double dt = 0.001;
    for (double t = delay_; t > 0; t -= dt) {
      int i = std::ceil(t / dt_);
      VectorU input = historyInput_[history_length_ - i];
      step(x0_delay, input, dt);
    }
    return x0_delay;
  }
  //zy changed end

 public:
  MpcCar(ros::NodeHandle& nh) : nh_(nh) {
    // load parameters
    nh.getParam("ll", ll_);
    nh.getParam("dt", dt_);
    //rho infers v
    //nh.advertise
    nh.getParam("rho", rho_);
    nh.getParam("rho2", rho2_);
    nh.getParam("N", N_);
    nh.getParam("rhoN", rhoN_);
    nh.getParam("v_max", v_max_);
    nh.getParam("a_max", a_max_);
    nh.getParam("delta_max", delta_max_);
    nh.getParam("ddelta_max", ddelta_max_);
    nh.getParam("delay", delay_);
    nh.getParam("delay_steer", delay_steer_);
    nh.getParam("kp_v", kp_v_);
    nh.getParam("ki_v", ki_v_);
    nh.getParam("kd_v", kd_v_);
    nh.getParam("kp_phi", kp_phi_);
    nh.getParam("ki_phi", ki_phi_);
    nh.getParam("kd_phi", kd_phi_);

    double kp_v_, ki_v_, kd_v_;
    double kp_phi_, ki_phi_, kd_phi_;
    history_length_ = std::ceil(delay_ / dt_);

    ref_pub_ = nh.advertise<nav_msgs::Path>("reference_path", 1);
    traj_pub_ = nh.advertise<nav_msgs::Path>("traj", 1);
    traj_delay_pub_ = nh.advertise<nav_msgs::Path>("traj_delay", 1);

    // set predict mats size
    predictState_.resize(N_);
    predictInput_.resize(N_);
    trajectory_state_.resize(N_);


    for (int i = 0; i < N_; ++i) {
      predictInput_[i].setZero();
    }
    for (int i = 0; i < history_length_; ++i) {
      historyInput_.emplace_back(0, 0);
    }

    Ad_.setIdentity();  // Ad for instance
    Bd_.setZero();
    Bd_(3, 0) = dt_;
    gd_.setZero();
    // set size of sparse matrices
    P_.resize(m * N_, m * N_);
    q_.resize(m * N_, 1);
    Qx_.resize(n * N_, n * N_);
    // stage cost
    Qx_.setIdentity();
    for (int i = 1; i < N_; ++i) {
      Qx_.coeffRef(i * n - 2, i * n - 2) = rho_;
      Qx_.coeffRef(i * n - 1, i * n - 1) = 0;
    }
    Qx_.coeffRef(N_ * n - 4, N_ * n - 4) = rhoN_;
    Qx_.coeffRef(N_ * n - 3, N_ * n - 3) = rhoN_;
    Qx_.coeffRef(N_ * n - 2, N_ * n - 2) = rhoN_ * rho_;
    int n_cons = 4;  // v a delta ddelta
    A_.resize(n_cons * N_, m * N_);
    l_.resize(n_cons * N_, 1);
    u_.resize(n_cons * N_, 1);
    // v constrains
    Cx_.resize(1 * N_, n * N_);
    lx_.resize(1 * N_, 1);
    ux_.resize(1 * N_, 1);
    // a delta constrains
    Cu_.resize(3 * N_, m * N_);
    lu_.resize(3 * N_, 1);
    uu_.resize(3 * N_, 1);
    // set lower and upper boundaries
    for (int i = 0; i < N_; ++i) {
      // TODO: set stage constraints of inputs (a, delta, ddelta)
      // -a_max <= a <= a_max for instance:
      Cu_.coeffRef(i * 3 + 0, i * m + 0) = 1;
      Cu_.coeffRef(i * 3 + 1, i * m + 1) = 1;
      Cu_.coeffRef(i * 3 + 2, i * m + 1) = 1;
      lu_.coeffRef(i * 3 + 0, 0) = -a_max_;
      uu_.coeffRef(i * 3 + 0, 0) = a_max_;
      lu_.coeffRef(i * 3 + 1, 0) = -delta_max_;
      uu_.coeffRef(i * 3 + 1, 0) = delta_max_;
      lu_.coeffRef(i * 3 + 2, 0) = -ddelta_max_ * dt_;
      uu_.coeffRef(i * 3 + 2, 0) = ddelta_max_ * dt_;
      if (i > 0) {
        Cu_.coeffRef(i * 3 + 2, (i - 1) * m + 1) = -1;
      }

      // TODO: set stage constraints of states (v)
      // -v_max <= v <= v_max
      Cx_.coeffRef(i, i * n + 3) = 1;
      lx_.coeffRef(i, 0) = -0.1;
      ux_.coeffRef(i, 0) = v_max_;
    }
 
  }

  void setRefTraj(const car_msgs::CarTrajS::ConstPtr& msg)
  {
    std::vector<plan_utils::Trajectory>().swap(traj_);
    std::vector<int>().swap(traj_singul_);
    //@zy
    //to process the direction of the multi-segment trajectory
    for(int i = 0; i < msg->singul.size(); i++){
      traj_singul_.push_back(msg->singul[i]);
    }
    //to receive multi-segment trajectory
    for(int i = 0; i < msg->size ; i++){
      int piece_nums = msg->trajvector[i].duration.size();
      std::vector<double> dura(piece_nums);
      std::vector<plan_utils::CoefficientMat> cMats(piece_nums);
      for (int j = 0; j < piece_nums; ++j) {
        int j6 = j * 6;
        cMats[j].row(0) << msg->trajvector[i].coef_x[j6 + 0], msg->trajvector[i].coef_x[j6 + 1], msg->trajvector[i].coef_x[j6 + 2],
            msg->trajvector[i].coef_x[j6 + 3], msg->trajvector[i].coef_x[j6 + 4], msg->trajvector[i].coef_x[j6 + 5];
        cMats[j].row(1) << msg->trajvector[i].coef_y[j6 + 0], msg->trajvector[i].coef_y[j6 + 1], msg->trajvector[i].coef_y[j6 + 2],
            msg->trajvector[i].coef_y[j6 + 3], msg->trajvector[i].coef_y[j6 + 4], msg->trajvector[i].coef_y[j6 + 5];
        dura[j] = msg->trajvector[i].duration[j];
      }
      int s;
      s = msg->singul[i];
      traj_.push_back( plan_utils::Trajectory(dura, cMats, s) );
    }
  }

  void getState(Eigen::VectorXd& state, const double& t, int index)
  {
    state.head(2) = traj_.at(index).getPos(t);  // px py
    state(2) = traj_.at(index).getAngle(t);  // phi
    state(3) = traj_.at(index).getVel(t);  // v
    state(4) = traj_.at(index).getSteer(t); // delta
    state(5) = traj_singul_[index];
  }


  /*
    * function setTrackingPoint
    * param t: current duration from traj_start_time 
    * return: a list of points for visualization
  */
  std::vector<Eigen::Vector2d> setTrackingPoint(const double& t)
  {
    std::vector<Eigen::Vector2d> ctrl_list;
    double t0 = t + delay_steer_;

    int tmp_exe_index = 0;
    double starttime = 0;
    //@zy
    //locate the index for the t0
    for(int i = 0; ;i++){
      if(t0 < starttime + traj_.at(tmp_exe_index).getTotalDuration())
        break;
      else{
        starttime += traj_.at(tmp_exe_index).getTotalDuration();
        tmp_exe_index++;
      }
    }
    //get the status of each interval
    for(int i = 0; i < N_; i++)
    {
      Eigen::VectorXd state; // x y yaw cur v
      state.resize(6);
      double t_state;
      double t_cmd = i * dt_ + t0;

      if(t_cmd < starttime + traj_.at(tmp_exe_index).getTotalDuration())
      {
        t_state = t_cmd - starttime;
        getState(state, t_state, tmp_exe_index);
      }
      else if(t_cmd >= starttime + traj_.at(tmp_exe_index).getTotalDuration() && tmp_exe_index < traj_.size() - 1)
      {

        starttime += traj_.at(tmp_exe_index).getTotalDuration();
        t_state = t_cmd - starttime;
        tmp_exe_index++; 
        getState(state, t_state, tmp_exe_index);
      }
      else // reach the end
      {
        t_state = traj_.at(tmp_exe_index).getTotalDuration();
        getState(state, t_state, tmp_exe_index);
      }
      trajectory_state_[i] = state;
      ctrl_list.emplace_back(state.head(2));
    }

    return ctrl_list;
  }

  double getObjectRho(){
      return this->rho_;
  }
  double getObjectRho2(){
      return this->rho2_;
  }
  double getObjectll(){
      return this->ll_;
  }
  
  VectorX setTrackingPoint2(const double& t)
  {
    double t0 = t ;
    VectorX retstate;
    Eigen::VectorXd state;
    state.resize(6);
    getState(state, t0, 0);
    retstate.head(3) = state.head(3);
    return retstate;
  }

  double trajDuration(int i)
  {
    return traj_.at(i).getTotalDuration();
  }


  void getPredictXU(double t, VectorX& state, VectorU& input) {
    if (t <= dt_) {
      state = predictState_.front();
      input = predictInput_.front();
      return;
    }
    int horizon = std::floor(t / dt_);
    double dt = t - horizon * dt_;
    state = predictState_[horizon - 1];
    input = predictInput_[horizon - 1];
    double phi = state(2);
    double v = state(3);
    double a = input(0);
    double delta = input(1);
    state(0) += dt * v * cos(phi);
    state(1) += dt * v * sin(phi);
    state(2) += dt * v / ll_ * tan(delta);
    state(3) += dt * a;
  }

  std::vector<Eigen::Vector2d> getPredictPoints()
  {
    std::vector<Eigen::Vector2d> predpoints;
    predpoints.clear();
    for(int i = 0; i < predictState_.size(); i++)
      predpoints.push_back(predictState_[i].head(2));

    return predpoints;
  }

  void forward(const VectorX& xk,
               const VectorU& uk,
               VectorX& xk_1) {
    // const auto& x = xk(0);
    // const auto& y = xk(1);
    const auto& phi = xk(2);
    const auto& v = xk(3);
    const auto& a = uk(0);
    const auto& delta = uk(1);
    xk_1 = xk;
    xk_1(0) += v * std::cos(phi) * dt_;
    xk_1(1) += v * std::sin(phi) * dt_;
    xk_1(2) += v / ll_ * std::tan(delta) * dt_;
    xk_1(3) += a * dt_;
  }

  void getCmd(Eigen::Vector2d& cmd) // v and steer
  {
    double steer = predictInput_.front()(1);
    VectorX state = predictState_[ceil((delay_ - delay_steer_) / dt_)];
    double vel = state(3);
    cmd(0) = vel;
    cmd(1) = steer;
    std::cout<<"vel: " << vel << " steer: " << steer << std::endl; 
  }

  void backward(const VectorX& xk,
                const VectorU& uk,
                const VectorX& grad_xk_1,
                VectorX& grad_xk,
                VectorU& grad_uk) {
    // const auto& x = xk(0);
    // const auto& y = xk(1);
    const auto& phi = xk(2);
    const auto& v = xk(3);
    // const auto& a = uk(0);
    const auto& delta = uk(1);
    const auto& grad_x_1 = grad_xk_1(0);
    const auto& grad_y_1 = grad_xk_1(1);
    const auto& grad_phi_1 = grad_xk_1(2);
    const auto& grad_v_1 = grad_xk_1(3);
    // auto& grad_x = grad_xk(0);
    // auto& grad_y = grad_xk(1);
    auto& grad_phi = grad_xk(2);
    auto& grad_v = grad_xk(3);
    auto& grad_a = grad_uk(0);
    auto& grad_delta = grad_uk(1);
    grad_xk = grad_xk_1;
    grad_uk.setZero();
    grad_v += grad_x_1 * std::cos(phi) * dt_;
    grad_phi += grad_x_1 * v * (-std::sin(phi)) * dt_;
    grad_v += grad_y_1 * std::sin(phi) * dt_;
    grad_phi += grad_y_1 * v * std::cos(phi) * dt_;
    grad_v += grad_phi_1 * std::tan(delta) / ll_ * dt_;
    grad_delta += grad_phi_1 * v / ll_ / std::cos(delta) / std::cos(delta) * dt_;
    grad_a += grad_v_1 * dt_;
  }

  double box_constrant(const double& x,
                       const double& l,
                       const double& u,
                       double& grad) {
    double rho = 1e4;
    double lpen = l - x;
    double upen = x - u;
    if (lpen > 0) {
      double lpen2 = lpen * lpen;
      grad = -rho * 3 * lpen2;
      return rho * lpen2 * lpen;
    } else if (upen > 0) {
      double upen2 = upen * upen;
      grad = rho * 3 * upen2;
      return rho * upen2 * upen;
    } else {
      grad = 0;
      return 0;
    }
  }

  double stage_cost_gradient(const int& k,
                             const VectorX& x,
                             VectorX& grad_x) {
    const Eigen::Vector3d& x_r = reference_states_[k];
    Eigen::Vector3d dx = x.head(3) - x_r;

    //zy changed
    grad_x.head(3) = 2 * dx;
    double a_x = dx(0) * std::cos(x_r(2)) + dx(1) * std::sin(x_r(2));
    double a_y = (-1) * dx(0) * std::sin(x_r(2)) + dx(1) * std::cos(x_r(2));
    
    grad_x(0) = 2 * a_x * std::cos(x_r(2)) + 2 * Rho2 * a_y * (-1) * std::sin(x_r(2));
    grad_x(1) = 2 * a_x * std::sin(x_r(2)) + 2 * Rho2 * a_y * std::cos(x_r(2));
    grad_x(2) = 2 * Rho * dx(2);
    //zy changed end
    grad_x(3) = 0;

    //zy changed
    double cost = dx.squaredNorm();
    cost = a_x * a_x + Rho2 * a_y * a_y + Rho * dx(2) * dx(2);
    //zy changed end

    // TODO: penalty constraints
    double grad_v = 0;
    // cost += box_constrant(x(3), -0.1, v_max_, grad_v);
    if(reference_states_singul_[k] == 1)
      cost += box_constrant(x(3), -0.1, v_max_, grad_v);
    else
      cost += box_constrant(x(3), -v_max_, 0.1, grad_v);
    grad_x(3) += grad_v;
    return cost;
  }

  static inline double objectiveFunc(void* ptrObj,
                                     const double* x,
                                     double* grad,
                                     const int n) {
    // std::cout << "\033[32m ************************************** \033[0m" << std::endl;
    MpcCar& obj = *(MpcCar*)ptrObj;
    Eigen::Map<const Eigen::MatrixXd> inputs(x, m, obj.N_);
    Eigen::Map<Eigen::MatrixXd> grad_inputs(grad, m, obj.N_);

    // forward propogate
    std::vector<VectorX> states(obj.N_ + 1);
    states[0] = obj.x0_observe_;
    VectorX xk_1 = obj.x0_observe_;
    for (int i = 0; i < obj.N_; ++i) {
      obj.forward(states[i], inputs.col(i), xk_1);
      states[i + 1] = xk_1;
    }
    // cost and gradient of states
    double total_cost = 0;
    VectorX grad_xk, grad_xk_1;
    VectorU grad_uk;
    grad_xk.setZero();
    for (int i = obj.N_ - 1; i >= 0; i--) {
      total_cost += obj.stage_cost_gradient(i, states[i + 1], grad_xk_1);
      grad_xk_1 = grad_xk_1 + grad_xk;
      obj.backward(states[i], inputs.col(i), grad_xk_1, grad_xk, grad_uk);
      grad_inputs.col(i) = grad_uk;
    }
    // cost and gradient of inputs
    for (int i = 0; i < obj.N_; ++i) {
      double a = inputs.col(i)(0);
      double delta = inputs.col(i)(1);
      double grad_a, grad_delta;
      total_cost += obj.box_constrant(a, -obj.a_max_, obj.a_max_, grad_a);
      grad_inputs.col(i)(0) += grad_a;
      total_cost += obj.box_constrant(delta, -obj.delta_max_, obj.delta_max_, grad_delta);
      grad_inputs.col(i)(1) += grad_delta;
    }
    for (int i = 0; i < obj.N_ - 1; ++i) {
      double delta_k = inputs.col(i)(1);
      double delta_k_1 = inputs.col(i + 1)(1);
      double ddelta = delta_k_1 - delta_k;
      double grad_ddelta;
      total_cost += obj.box_constrant(ddelta,
                                      -obj.ddelta_max_ * obj.dt_,
                                      obj.ddelta_max_ * obj.dt_,
                                      grad_ddelta);
      grad_inputs.col(i)(1) -= grad_ddelta;
      grad_inputs.col(i + 1)(1) += grad_ddelta;
    }
    // bug here unresolved
    // for(int i = 0; i * obj.dt_ < obj.delay_- obj.delay_steer_; i++){
    //   grad_inputs.col(i)(0) = 0;
    // }
    return total_cost;
  }

  int solveNMPC(const VectorX& x0_observe) {

    historyInput_.pop_front();
    historyInput_.push_back(predictInput_.front());
    x0_observe_ = compensateDelay(x0_observe);
    reference_states_.resize(N_);
    reference_states_singul_.resize(N_);
    double phi, last_phi = x0_observe_(2);
    
    for (int i = 0; i < N_; ++i) {
      phi = trajectory_state_[i](2);
      if (phi - last_phi > M_PI) {
        while(phi - last_phi > M_PI)
          phi -= 2 * M_PI;
      } else if (phi - last_phi < -M_PI) {
        while(phi - last_phi < -M_PI) 
        phi += 2 * M_PI;
      }
      last_phi = phi;
      reference_states_[i] = Eigen::Vector3d(trajectory_state_[i](0), trajectory_state_[i](1), phi);
      reference_states_singul_[i] = trajectory_state_[i](5);
    }

    double* x = new double[m * N_];
    Eigen::Map<Eigen::MatrixXd> inputs(x, m, N_);
    inputs.setZero();

    int num = ceil(delay_steer_ / dt_) ;
    // bug here unresolved
    // for (int i = 0; i * dt_ < delay_ - delay_steer_; i++ ){
    //   // std::cout << "vector length: " << historyInput_.size() << " index: " << num + i << std::endl;
    //   VectorU input1 = historyInput_[num + i]; //+1?
    //   // VectorU input2 = historyInput_[history_length_ - i];
    //   // VectorU input;
    //   inputs(0, i)=input1(0);
    //   // input(1)=input2(1);
    // }

    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;
    lbfgs_params.past = 3;
    lbfgs_params.g_epsilon = 0.0;
    lbfgs_params.min_step = 1e-32;
    lbfgs_params.delta = 1e-5;
    lbfgs_params.line_search_type = 0;

    double minObjective;
    auto ret = lbfgs::lbfgs_optimize(m * N_, x, &minObjective, &objectiveFunc, nullptr, nullptr, this, &lbfgs_params);
    // std::cout << "\033[32m"
    //           << "ret: " << ret << "\033[0m" << std::endl;
    VectorX xk = x0_observe_, xk_1;
    int delay_step = ceil((delay_ - delay_steer_) / dt_);
    predictState_[0] = x0_observe_;

    for (int i = 0; i < N_ ; ++i) {
      predictInput_[i](0) = inputs.col(i)(0);
      predictInput_[i](1) = inputs.col(i)(1);
      forward(xk, inputs.col(i), xk_1);
      predictState_[i+1] = xk_1;
      xk = xk_1;
    }

    std::cout << "ret: " << ret <<"-------------------------------------------"<< std::endl;
    return ret;
  } 
  double calTargetIndex(std::vector<double>robot_state, std::vector<std::vector<double>>refer_path){
    std::vector<double>dists;
    for (std::vector<double>xy:refer_path) {
        double dist = sqrt(pow(xy[0]-robot_state[0],2)+pow(xy[1]-robot_state[1],2));
        dists.push_back(dist);
    }
    return min_element(dists.begin(),dists.end())-dists.begin(); //返回vector最小元素的下标
}


  int solvePID(const VectorX& x0_observe) {
    historyInput_.pop_front();
    historyInput_.push_back(predictInput_.front());
    x0_observe_ = compensateDelay(x0_observe);
    reference_states_.resize(N_);
    double phi, last_phi = x0_observe_(2);

    phi = trajectory_state_[0](2);
    if (phi - last_phi > M_PI) {
      phi -= 2 * M_PI;
    } else if (phi - last_phi < -M_PI) {
      phi += 2 * M_PI;
    }
    Eigen::VectorXd ref_state = Eigen::Vector3d(trajectory_state_[0](0), trajectory_state_[0](1), phi);

    //  kp_v_, ki_v_, kd_v_;
    //  kp_phi_, ki_phi_, kd_phi_;
    VectorX x = x0_observe_, xk_1;
    double dis = (x.head(2) - ref_state.head(2)).norm();
    phi = x(2);
    double e_long, e_latt, de_long, de_latt;
    double theta = atan2(ref_state(1) - x(1), ref_state(0) - x(0)) - phi;
    double e_angle = ref_state(2) - phi;
    e_long = dis * cos(theta);  // =-cos(phi) * (x(0) - ref_state(0)) - sin(phi) * (x(1) - ref_state(1));
    e_latt = dis * sin(theta) + e_angle;  // = sin(phi) * (x(0) - ref_state(0)) - cos(phi) * (x(1) - ref_state(1));
    de_long = -x(3);  // x(0)_dot = v * cos(phi)
    de_latt = 0.0;  // -v * tan(delta) / L, but we don't know delta 
    // std::cout << e_long << "  " << de_long << std::endl;
    // std::cout << e_latt << "  " << e_latt << std::endl;
    double v = kp_v_ * e_long + kd_v_ * de_long;
    double delta = kp_phi_ * e_latt + kd_phi_ * de_latt;

    v = v > v_max_ ? v_max_ : v;
    v = v < -v_max_ ? -v_max_ : v;
    delta = delta > delta_max_ ? delta_max_ : delta;
    delta = delta < -delta_max_ ? -delta_max_ : delta;
    
    predictInput_[0] = Eigen::Vector2d(v, delta);
    forward(x, predictInput_[0], xk_1);
    predictState_[0] = xk_1;
    return 1;
  }

  void linearization(const double& phi,
                     const double& v,
                     const double& delta) {
    // TODO: set values to Ad_, Bd_, gd_
    Ad_(0, 2) = -v * sin(phi) * dt_;
    Ad_(0, 3) = cos(phi) * dt_;
    Ad_(1, 2) = v * cos(phi) * dt_;
    Ad_(1, 3) = sin(phi) * dt_;
    Ad_(2, 3) = tan(delta) / ll_ * dt_;
    Bd_(2, 1) = v / ll_ / cos(delta) / cos(delta) * dt_;
    gd_(0) = v * sin(phi) * dt_ * phi;
    gd_(1) = -v * cos(phi) * dt_ * phi;
    gd_(2) = -v / ll_ / cos(delta) / cos(delta) * dt_ * delta;
    return;
  }

  // int solveQP(const VectorX& x0_observe) {
  //   x0_observe_ = x0_observe;
  //   historyInput_.pop_front();
  //   historyInput_.push_back(predictInput_.front());
  //   lu_.coeffRef(2, 0) = predictInput_.front()(1) - ddelta_max_ * dt_;
  //   uu_.coeffRef(2, 0) = predictInput_.front()(1) + ddelta_max_ * dt_;
  //   VectorX x0 = compensateDelay(x0_observe_);
  //   // set BB, AA, gg
  //   Eigen::MatrixXd BB, AA, gg;
  //   BB.setZero(n * N_, m * N_);
  //   AA.setZero(n * N_, n);
  //   gg.setZero(n * N_, 1);
  //   double phi, v, delta;
  //   double last_phi = x0(2);
  //   Eigen::SparseMatrix<double> qx;
  //   qx.resize(n * N_, 1);
  //   for (int i = 0; i < N_; ++i) {
  //     phi = trajectory_state_[i](2);
  //     v = trajectory_state_[i](3);
  //     delta = trajectory_state_[i](4);
  //     // phi, v, delta at t
  //     if (phi - last_phi > M_PI) {
  //       phi -= 2 * M_PI;
  //     } else if (phi - last_phi < -M_PI) {
  //       phi += 2 * M_PI;
  //     }
  //     last_phi = phi;
  //     if (init_) {
  //       double phii = predictState_[i](2);
  //       v = predictState_[i](3);
  //       delta = predictInput_[i](1);
  //       if (phii - last_phi > M_PI) {
  //         phii -= 2 * M_PI;
  //       } else if (phii - last_phi < -M_PI) {
  //         phii += 2 * M_PI;
  //       }
  //       last_phi = phii;
  //       linearization(phii, v, delta);
  //     } else {
  //       linearization(phi, v, delta);
  //     }
  //     // calculate big state-space matrices
  //     /* *                BB                AA
  //      * x1    /       B    0  ... 0 \    /   A \
  //      * x2    |      AB    B  ... 0 |    |  A2 |
  //      * x3  = |    A^2B   AB  ... 0 |u + | ... |x0 + gg
  //      * ...   |     ...  ...  ... 0 |    | ... |
  //      * xN    \A^(n-1)B  ...  ... B /    \ A^N /
  //      *
  //      *     X = BB * U + AA * x0 + gg
  //      * */
  //     if (i == 0) {
  //       BB.block(0, 0, n, m) = Bd_;
  //       AA.block(0, 0, n, n) = Ad_;
  //       gg.block(0, 0, n, 1) = gd_;
  //     } else {
  //       // TODO: set BB AA gg
  //       BB.block(n * i, 0, n, m * N_) = Ad_ * BB.block(n * (i - 1), 0, n, m * N_);
  //       BB.block(n * i, m * i, n, m) = Bd_;
  //       AA.block(n * i, 0, n, n) = Ad_ * AA.block(n * (i - 1), 0, n, n);
  //       gg.block(n * i, 0, n, 1) = Ad_ * gg.block(n * (i - 1), 0, n, 1) + gd_;
  //     }
  //     // TODO: set qx
  //     Eigen::Vector2d xy = trajectory_state_[i].head(2); // reference (x_r, y_r)
  //     qx.coeffRef(i * n + 0, 0) = -xy.x();
  //     qx.coeffRef(i * n + 1, 0) = -xy.y();
  //     qx.coeffRef(i * n + 2, 0) = -rho_ * phi;
  //     // std::cout << "phi[" << i << "]: " << phi << std::endl;
  //     if (i == N_ - 1) {
  //       qx.coeffRef(i * n + 0, 0) *= rhoN_;
  //       qx.coeffRef(i * n + 1, 0) *= rhoN_;
  //       qx.coeffRef(i * n + 2, 0) *= rhoN_;
  //     }
  //   }
  //   Eigen::SparseMatrix<double> BB_sparse = BB.sparseView();
  //   Eigen::SparseMatrix<double> AA_sparse = AA.sparseView();
  //   Eigen::SparseMatrix<double> gg_sparse = gg.sparseView();
  //   Eigen::SparseMatrix<double> x0_sparse = x0.sparseView();

  //   Eigen::SparseMatrix<double> Cx = Cx_ * BB_sparse;
  //   Eigen::SparseMatrix<double> lx = lx_ - Cx_ * AA_sparse * x0_sparse - Cx_ * gg_sparse;
  //   Eigen::SparseMatrix<double> ux = ux_ - Cx_ * AA_sparse * x0_sparse - Cx_ * gg_sparse;
  //   Eigen::SparseMatrix<double> A_T = A_.transpose();
  //   A_T.middleCols(0, Cx.rows()) = Cx.transpose();
  //   A_T.middleCols(Cx.rows(), Cu_.rows()) = Cu_.transpose();
  //   A_ = A_T.transpose();
  //   for (int i = 0; i < lx.rows(); ++i) {
  //     l_.coeffRef(i, 0) = lx.coeff(i, 0);
  //     u_.coeffRef(i, 0) = ux.coeff(i, 0);
  //   }
  //   for (int i = 0; i < lu_.rows(); ++i) {
  //     l_.coeffRef(i + lx.rows(), 0) = lu_.coeff(i, 0);
  //     u_.coeffRef(i + lx.rows(), 0) = uu_.coeff(i, 0);
  //   }
  //   Eigen::SparseMatrix<double> BBT_sparse = BB_sparse.transpose();
  //   P_ = BBT_sparse * Qx_ * BB_sparse;
  //   q_ = BBT_sparse * Qx_.transpose() * (AA_sparse * x0_sparse + gg_sparse) + BBT_sparse * qx;
  //   // osqp
  //   Eigen::VectorXd q_d = q_.toDense();
  //   Eigen::VectorXd l_d = l_.toDense();
  //   Eigen::VectorXd u_d = u_.toDense();
  //   qpSolver_.setMats(P_, q_d, A_, l_d, u_d);
  //   qpSolver_.solve();
  //   int ret = qpSolver_.getStatus();
  //   if (ret != 1) {
  //     ROS_ERROR("fail to solve QP!");
  //     return ret;
  //   }
  //   Eigen::VectorXd sol = qpSolver_.getPrimalSol();
  //   Eigen::MatrixXd solMat = Eigen::Map<const Eigen::MatrixXd>(sol.data(), m, N_);
  //   Eigen::VectorXd solState = BB * sol + AA * x0 + gg;
  //   Eigen::MatrixXd predictMat = Eigen::Map<const Eigen::MatrixXd>(solState.data(), n, N_);

  //   for (int i = 0; i < N_; ++i) {
  //     predictInput_[i] = solMat.col(i);
  //     predictState_[i] = predictMat.col(i);
  //   }
  //   init_ = true;
  //   return ret;
  // }


  // visualization
  void visualization() {
    nav_msgs::Path msg;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped p;
    for (int i = 0; i < N_ ; i++){
      p.pose.position.x = trajectory_state_[i](0);
      p.pose.position.y = trajectory_state_[i](1);
      p.pose.position.z = 0.0;
      msg.poses.push_back(p);
    }
    ref_pub_.publish(msg);
    msg.poses.clear();
    for (int i = 0; i < N_; ++i) {
      p.pose.position.x = predictState_[i](0);
      p.pose.position.y = predictState_[i](1);
      p.pose.position.z = 0.0;
      msg.poses.push_back(p);
    }
    traj_pub_.publish(msg);
    msg.poses.clear();
    VectorX x0_delay = x0_observe_;
    double dt = 0.001;
    for (double t = delay_; t > 0; t -= dt) {
      int i = std::ceil(t / dt_);
      VectorU input = historyInput_[history_length_ - i];
      step(x0_delay, input, dt);
      p.pose.position.x = x0_delay(0);
      p.pose.position.y = x0_delay(1);
      p.pose.position.z = 0.0;
      msg.poses.push_back(p);
    }
    traj_delay_pub_.publish(msg);
  }
};

}  // namespace mpc_car