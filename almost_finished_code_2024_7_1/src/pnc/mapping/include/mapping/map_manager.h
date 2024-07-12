#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H


#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mapping/mapping.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <car_msgs/OccMap3d.h>
#include <vehicle_params/vehicle_param.h>
#include <mapping/raycast.h>

#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <string>
#include <queue>

#include <atomic>
#include <thread>

#define INVALID_IDX -1

namespace mapping {

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
    ImageOdomSyncPolicy;
typedef message_filters::Synchronizer<ImageOdomSyncPolicy>
    ImageOdomSynchronizer;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>
    CloudOdomSyncPolicy;
typedef message_filters::Synchronizer<CloudOdomSyncPolicy>
    CloudOdomSynchronizer;

struct CamConfig {
  // camera paramters
  double rate;
  double range;
  int width;
  int height;
  double fx;
  double fy;
  double cx;
  double cy;
  double depth_scaling_factor;
};

class MapManager {
 private:
  // 0. common parameters
  std::thread initThread_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  ros::Publisher gridmap_pub_, gridmap_inflate_pub_; 
  ros::Publisher global_view_cloud_pub_;
  bool map_recieved_ = false;
  int sensor_type_ = -1;
  Eigen::Vector3d map_size_, origin_;
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_, min_occupancy_log_;
  OccGridMap gridmap_;
  int inflate_size_;
  std::shared_ptr<VehicleParam> vehicle_param_;


  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicyPointcloud_Odom;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicyPointcloud_Odom>> pointcloud_odom_sync_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> Local_Pointcloud_sub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> Odometry_sub_;
  void OdometryAndPointcloud_cb(const sensor_msgs::PointCloud2::ConstPtr &pcl_msg, const nav_msgs::Odometry::ConstPtr &odom_msg);
  void globalOccVis_cb(const ros::TimerEvent& e);
  Eigen::Vector3i global_map_size_;  //global_map_size_ represents the number of the grids in each dircetion of the global map
  Eigen::Vector3d center_position_;
  double resolution_, resolution_inv_;
  double max_ray_length_, min_ray_length_;
  int grid_size_y_multiply_z_;
  int buffer_size_, buffer_size_2d_;
  int number_of_points_; //This parameter represents the number of points in each frame of the pointcloud
  std::vector<double> occupancy_buffer_; //This buffer stores the states of each grid in the "global" map
  std::vector<double> occupancy_buffer_2d_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr history_view_cloud_ptr_;
  
  /*-------------- map fusion ------------------*/
  std::vector<int> cache_all_, cache_hit_;
  std::vector<int> cache_traverse_, cache_rayend_;
  std::queue<Eigen::Vector3i> cache_voxel_;
  int raycast_num_;
  void raycastProcess(const Eigen::Vector3d& t_wc, const pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudTransformed);
  int setCacheOccupancy(const Eigen::Vector3d& pos, int occ);
  /*-------------- map fusion ------------------*/

  // 1. global map parameters
    // NOTE just for global map in simulation
  ros::Timer global_map_timer_;
  ros::Timer global_occ_vis_timer_;
  ros::Subscriber map_pc_sub_;
  std::string map_path_;

  // 2. depth parameters
  CamConfig camConfig_;  // just store the parameters of camera
  int down_sample_factor_;
  Eigen::Matrix3d cam2body_R_;
  Eigen::Vector3d cam2body_p_;
  
    // just for depth filter
  Eigen::Vector3d last_cam_p_;
  Eigen::Quaterniond last_cam_q_;
  bool get_first_frame_ = false;
  cv::Mat last_depth_;
  double depth_filter_tolerance_;
  double depth_filter_mindist_;
  int depth_filter_margin_;

  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  std::shared_ptr<ImageOdomSynchronizer> depth_odom_sync_Ptr_;  

  // 3. pointcloud parameters
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_;
  std::shared_ptr<CloudOdomSynchronizer> cloud_odom_sync_Ptr_;

  Eigen::Matrix3d lidar2body_R_;
  Eigen::Vector3d lidar2body_p_;

  // @NOTE global map 
  void map_call_back(const sensor_msgs::PointCloud2ConstPtr& msgPtr);
  void global_map_timer_callback(const ros::TimerEvent& event);
  //  @NOTE depth
  void depth_odom_callback(const sensor_msgs::ImageConstPtr& depth_msg,
                           const nav_msgs::OdometryConstPtr& odom_msg);
  // @NOTE cloud c
  void cloud_odom_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                          const nav_msgs::OdometryConstPtr& odom_msg);

  void map_odom_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                        const nav_msgs::OdometryConstPtr& odom_msg);

  void init(ros::NodeHandle& nh);
  
  void GetDenseVerticesOfOrientedBoundingBox(const Eigen::Vector3d &pose, const Eigen::Vector2d &shape, std::vector<Eigen::Vector2d> &vertices, double res=0.1);


 public:
  MapManager() {}
  MapManager(ros::NodeHandle &nh) {
    init(nh);
  }


  bool Initialized() {return map_recieved_;}

  bool isInMap(const Eigen::Vector3d &pos);
  bool isInMap(const Eigen::Vector3i &id);
  bool isInMap2d(const Eigen::Vector2i &pos);
  bool isInMap2d(const Eigen::Vector2d &id);
  void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  void indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos);
  void posToIndex2d(const Eigen::Vector2d& pos, Eigen::Vector2i& id);
  void indexToPos2d(const Eigen::Vector2i& id, Eigen::Vector2d& pos);

  int getVoxelState(const Eigen::Vector3d &pos);
  int getVoxelState(const Eigen::Vector3i &id);
  int getVoxelState2d(const Eigen::Vector2d &pos);
  int getVoxelState2d(const Eigen::Vector2i &id);

  // bool checkOccupied(const Eigen::Vector3d &p){return gridmap_.isOccupied(p);} 
  bool CheckCollisionUsingGlobalPosition(const Eigen::Vector2d &p);
  bool CheckIfCollisionUsingPosAndYaw(const Eigen::Vector3d &state);
  /*
    * @brief check if collision by vehicle shape
    * @return true if the pose is collision
  */
  bool CheckIfCollisionUsingLine(const Eigen::Vector2d p1, const Eigen::Vector2d p2, double checkl);
  // double GetMapResolution() {return gridmap_.resolution;}
  double GetMapResolution() {return resolution_;}

  typedef std::shared_ptr<MapManager> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


} // namespace mapping


#endif