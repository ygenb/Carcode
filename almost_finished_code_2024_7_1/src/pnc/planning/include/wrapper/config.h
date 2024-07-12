#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <vector>

#include <ros/ros.h>

struct Config
{
    // @NOTE Vehicle flatness optimization part
    int traj_resolution_; // number of distinctive constrain points each piece
    int destraj_resolution_; //number of distinctive constrain points of the first and last piece (should be more dense!
    double wei_obs_;                         // obstacle weight
    double wei_surround_;                       // surround weight
    double wei_feas_;                        // feasibility weight
    double wei_sqrvar_;                      // squared variance weight
    double wei_time_;                        // time weight
    double surround_clearance_; // safe distance
    double half_margin;                        // safe margin
    double max_forward_vel, max_backward_vel, max_forward_cur, max_backward_cur;
    double max_forward_acc, max_backward_acc, max_phidot_, max_latacc_;       // dynamic limits
    double non_sinv = 0.1;
    bool GearOpt;
    int memsize;
    int past;
    double delta;
    double mini_T;


    static void loadParameters(Config &conf, const ros::NodeHandle &nh_priv)
    {
        // @NOTE Vehicle flatness optimization part
        nh_priv.getParam("traj_res", conf.traj_resolution_);
        nh_priv.getParam("dense_traj_res", conf.destraj_resolution_);
        nh_priv.getParam("wei_sta_obs", conf.wei_obs_);
        nh_priv.getParam("wei_dyn_obs", conf.wei_surround_);
        nh_priv.getParam("wei_feas", conf.wei_feas_);
        nh_priv.getParam("wei_sqrvar", conf.wei_sqrvar_);
        nh_priv.getParam("wei_time", conf.wei_time_);
        nh_priv.getParam("dyn_obs_clearance", conf.surround_clearance_);
        nh_priv.getParam("half_margin", conf.half_margin);
        nh_priv.getParam("max_phidot", conf.max_phidot_);
        nh_priv.getParam("max_forward_vel", conf.max_forward_vel);
        nh_priv.getParam("max_backward_vel", conf.max_backward_vel);
        nh_priv.getParam("max_forward_cur", conf.max_forward_cur);
        nh_priv.getParam("max_backward_cur", conf.max_backward_cur);
        nh_priv.getParam("max_forward_acc", conf.max_forward_acc);
        nh_priv.getParam("max_backward_acc", conf.max_backward_acc);
        nh_priv.getParam("max_latacc", conf.max_latacc_);
        nh_priv.getParam("max_nonsv", conf.non_sinv);
        nh_priv.getParam("GearOpt", conf.GearOpt);
        nh_priv.getParam("lbfgs_memsize", conf.memsize);
        nh_priv.getParam("lbfgs_past", conf.past);
        nh_priv.getParam("lbfgs_delta", conf.delta);
        nh_priv.getParam("mini_T", conf.mini_T);
    }
};

#endif