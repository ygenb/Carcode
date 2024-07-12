#ifndef _TRAJ_OPTIMIZER_H_
#define _TRAJ_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <ros/ros.h>

#include <plan_utils/traj_container.hpp>
// #include "minco_config.pb.h"

#include "traj_opt/lbfgs.hpp"
#include "plan_utils/geoutils2d.hpp"  
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vehicle_params/vehicle_param.h>
// #include "common/state/state.h"
// #include "plan_utils/traj_visualizer.h"
#include <wrapper/config.h>
#include "decomp_util/ellipsoid_decomp.h"
#include "decomp_ros_utils/data_ros_utils.h"


using namespace std;


class PolyTrajOptimizer
{

private:


  int traj_seg_num_;

  int traj_resolution_; // number of distinctive constrain points each piece
  int destraj_resolution_; //number of distinctive constrain points of the first and last piece (should be more dense!
  int variable_num_;     // optimization variables
  int iter_num_;         // iteration of the solver
  double min_ellip_dist2_; // min trajectory distance in surround


  enum FORCE_STOP_OPTIMIZE_TYPE
  {
    DONT_STOP,
    STOP_FOR_REBOUND,
    STOP_FOR_ERROR
  } force_stop_type_;

  /* optimization parameters */
  double wei_obs_;                         // obstacle weight
  double wei_surround_;                       // surround weight
  double wei_feas_;                        // feasibility weight
  double wei_sqrvar_;                      // squared variance weight
  double wei_time_;                        // time weight
  double surround_clearance_; // safe distance
  
  double half_margin;                        // safe margin
  VehicleParam veh_param_;          
  double t_now_;
  double L_;
  std::vector<Eigen::Vector2d> lz_set_;
  std::vector<Eigen::Vector2d> vec_le_, vec_lo_;
  int number_of_hyperplanes_of_ego_car_, number_of_hyperplanes_of_surround_car_;


  /*new added param*/
  double max_forward_vel, max_backward_vel, max_forward_cur, max_backward_cur;
  double max_forward_acc, max_backward_acc, max_phidot_, max_latacc_;       // dynamic limits
  double non_sinv = 0.1;
  bool GearOpt = true;
  /*lbfgs param*/
  int memsize;
  int past;
  double delta;
  double mini_T = 0.05;


  // Each col of cfgHs denotes a facet (outter_normal^T,point^T)^T
  Eigen::Matrix<double, 2, 2> B_h;
  double epis;
  std::vector<int> singul_container;
  std::vector<Eigen::MatrixXd> iniState_container;
  std::vector<Eigen::MatrixXd> finState_container;
  std::vector<std::vector<Eigen::MatrixXd>> cfgHs_container;
  std::vector<double> gearVel_container;
  int trajnum;
  /*debug*/
  Eigen::MatrixXd ctrl_points_;
  std::vector<Eigen::Vector2d> cos_points;
  std::vector<Eigen::Vector2d> key_points;
  std::vector<Eigen::MatrixXd> debug_hPolys;
  std::vector<plan_utils::MinJerkOpt> jerkOpt_container;
  std::vector<int> piece_num_container;

public:
  
  PolyTrajOptimizer() {}
  ~PolyTrajOptimizer() {}

  /* set variables */
  void setParam(ros::NodeHandle nh, Config cfg_);
  ros::Publisher debug_pub,debug_pub1,debug_galaxy_poly_pub_,debug_key_points_pub;
  void displayPoints();
  void displayCosPoints();
  void displayBugPoly();
  void displayKeyPoints();
  // void setEnvironment(const GridMap::Ptr &map);

  /* helper functions */
  inline const std::vector<plan_utils::MinJerkOpt> *getMinJerkOptPtr(void) { return &jerkOpt_container; }
  inline int get_traj_resolution_() { return traj_resolution_; };
  inline int get_destraj_resolution_() { return destraj_resolution_; };
  inline double getsurroundClearance(void) { return surround_clearance_; }

  /* main planning API */
  bool OptimizeTrajectory(const std::vector<Eigen::MatrixXd> &iniStates, const std::vector<Eigen::MatrixXd> &finStates,
                          std::vector<Eigen::MatrixXd> &initInnerPts, const Eigen::VectorXd &initTs,
                          std::vector<std::vector<Eigen::MatrixXd>> &hPoly_container,std::vector<int> singuls,double now = ros::Time::now().toSec(),double help_eps = 1.0e-4);


  double log_sum_exp(double alpha, Eigen::VectorXd &all_dists, double &exp_sum);

    /* optimization setup */
  bool setup(const Eigen::MatrixXd &iniState,
           const Eigen::MatrixXd &finState,
           const std::vector<Eigen::MatrixXd> &cfgPolyHs,
           Eigen::VectorXi &intervals,
           double &totalT,
           Eigen::MatrixXd &vecInP,
           std::vector<Eigen::MatrixXd> &polyH);


private:
  /* callbacks by the L-BFGS optimizer */
  static double costFunctionCallback(void *func_data, const Eigen::VectorXd &x, Eigen::VectorXd &grad);

  static int earlyExitCallback(void *func_data, const double *x, const double *g,
                                const double fx, const double xnorm, const double gnorm,
                                const double step, int n, int k, int ls);
  

  /* optimization setup */
  void gridMesh(const Eigen::Matrix3d &iState,
                const Eigen::Matrix3d &fState,
                const std::vector<Eigen::MatrixXd> &cfgPolyVs,
                const double &gridResolution,
                Eigen::VectorXi &intervalsVec);

  /* mappings between real world time and unconstrained virtual time */
  template <typename EIGENVEC>
  void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

  template <typename EIGENVEC>
  void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

  template <typename EIGENVEC, typename EIGENVECGD>
  void VirtualTGradCost(const Eigen::VectorXd &RT, const EIGENVEC &VT,
                        const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
                        double &costT);
  void VirtualTGradCost(const double &RT, const double & VT, const double & gdRT, double & gdVT, double & costT);


  void addPVAGradCost2CT(Eigen::VectorXd &costs, const int trajid, const double trajtime);


  double debugGradCheck(const int i_dp, // index of constraint point
                                        double t, // current absolute time
                                      Eigen::Vector2d sigma, // the rear model 
                                      Eigen::Vector2d dsigma,
                                      Eigen::Vector2d ddsigma,                                                                             
                                      const int trajid, const int sur_id, double res_t,Eigen::Matrix<double, 6, 2> c
                                      ,int i ,int j,double omg,double step,double wei_surround_,int K);

  /* @zhiwei vPolys are not used
  inline bool extractVs(const std::vector<Eigen::MatrixXd> &hPs,
                        std::vector<Eigen::MatrixXd> &vPs) const
  {
      const int M = hPs.size() - 1;

      vPs.clear();
      vPs.reserve(2 * M + 1);

      int nv;
      Eigen::MatrixXd curIH, curIV, curIOB;
      std::cout << "extractVs: 1" << std::endl;
      for (int i = 0; i < M; i++)
      {
          if (!geoutils::enumerateVs(hPs[i], curIV))
          {
              return false;
          }
          std::cout << "extractVs: 2" << std::endl;
          nv = curIV.cols();
          // curIOB.resize(3, nv); debug
          curIOB.resize(2, nv);
          curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
          vPs.push_back(curIOB);

          std::cout << "extractVs: 3" << std::endl;
          // curIH.resize(6, hPs[i].cols() + hPs[i + 1].cols());
          curIH.resize(4, hPs[i].cols() + hPs[i + 1].cols());
          curIH << hPs[i], hPs[i + 1];
          if (!geoutils::enumerateVs(curIH, curIV))
          {
              return false;
          }
          nv = curIV.cols();
          // curIOB.resize(3, nv);
          curIOB.resize(2, nv);
          curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
          vPs.push_back(curIOB);
      }

      std::cout << "extractVs: 5" << std::endl;
      if (!geoutils::enumerateVs(hPs.back(), curIV))
      {
          return false;
      }
      nv = curIV.cols();
      curIOB.resize(3, nv);
      curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
      vPs.push_back(curIOB);

      return true;
  }
  */

  inline bool extractWaypoints(const std::vector<Eigen::MatrixXd> &hPs,
                        std::vector<Eigen::Vector2d> &Pts) const
  {
      const int M = hPs.size() - 1;
      Pts.clear();
      Pts.reserve(M);

      Eigen::MatrixXd curIH;
      Eigen::Vector2d curPt;
      for(int i = 0; i < M; i++)
      {
        // union of two adjacent hPolys
        curIH.resize(4, hPs[i].cols() + hPs[i + 1].cols());
        curIH << hPs[i], hPs[i + 1];
        if(!geoutils::enumeratePts2D(curIH, curPt))
        {
          return false;
        }
        Pts.push_back(curPt);
      }
      return true;
  }

  std::vector<Eigen::MatrixXd> debug_P;
  std::vector<Eigen::VectorXd> debug_T;
  std::vector<double> debug_cost;

  void getBoundPts(Eigen::Vector2d &position, double angle, std::vector<Eigen::Vector2d> &BoundVertices);
  void positiveSmoothedL1(const double &x, double &f, double &df);
  void positiveSmoothedL3(const double &x, double &f, double &df);

public:
  typedef unique_ptr<PolyTrajOptimizer> Ptr;

};

#endif