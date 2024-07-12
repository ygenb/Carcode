#include <mapping/map_manager.h>

namespace mapping {

void MapManager::map_call_back(const sensor_msgs::PointCloud2ConstPtr& msgPtr) {
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromROSMsg(*msgPtr, point_cloud);
    if(point_cloud.size() == 0)
      ROS_WARN("[Map Manager]: point cloud size is 0?");
    // set free
    gridmap_.resetMap();
    // receive from map
    for (const auto& pt : point_cloud) {
      Eigen::Vector3d p(pt.x, pt.y, pt.z);
      // if((-map_size_.x() < p.x()) && (p.x() < map_size_.x()) && (-map_size_.y() < p.y() ) && (p.y() < map_size_.y()) && (-map_size_.z() < p.z() ) && (p.z() < map_size_.z())) {
      //   gridmap_.setOcc(p);
      // }
      gridmap_.setOcc(p);
    }

    // similar to cloud odom call back, need to time sync
    
    gridmap_.inflate(inflate_size_);

    ROS_WARN("[mapping] MAP REVIEVED!");
    map_recieved_ = true;
    return;
}

void MapManager::global_map_timer_callback(const ros::TimerEvent& event) {
    if (!map_recieved_) {
      return;
    }
    car_msgs::OccMap3d gridmap_msg;
    gridmap_.to_msg(gridmap_msg);
    gridmap_inflate_pub_.publish(gridmap_msg);
}

void MapManager::map_odom_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                                  const nav_msgs::OdometryConstPtr& odom_msg) {
    if (!map_recieved_) {
      map_recieved_ = true;
    }

    Eigen::Vector3d body_p(odom_msg->pose.pose.position.x,
                           odom_msg->pose.pose.position.y,
                           odom_msg->pose.pose.position.z);
    Eigen::Quaterniond body_q(
        odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
    Eigen::Vector3d sensor_p = body_q.toRotationMatrix() * lidar2body_p_ + body_p;
    // Eigen::Quaterniond sensor_q = body_q * Eigen::Quaterniond(lidar2body_R_);

    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromROSMsg(*cloud_msg, point_cloud);
    std::vector<Eigen::Vector3d> obs_pts;
    for(const auto& pts : point_cloud){
      Eigen::Vector3d p(pts.x, pts.y, pts.z);
      // @NOTE if the points are global data, otherwise they are in the lidar frame  
      // p = sensor_q * p + sensor_p;
      obs_pts.push_back(p);
    }
    
    gridmap_.updateByLocalMap(sensor_p, obs_pts);

    car_msgs::OccMap3d gridmap_msg, gridmap_inf_msg;
    gridmap_msg.header.frame_id = "world";
    gridmap_msg.header.stamp = ros::Time::now();
    gridmap_.to_msg(gridmap_msg);
    gridmap_pub_.publish(gridmap_msg);
    // inflate & pub
    gridmap_.inflate(inflate_size_);
    gridmap_.to_msg(gridmap_inf_msg);
    gridmap_inflate_pub_.publish(gridmap_inf_msg);
}

void MapManager::depth_odom_callback(const sensor_msgs::ImageConstPtr& depth_msg,
                           const nav_msgs::OdometryConstPtr& odom_msg) {
    ros::Time t1, t2;
    // t1 = ros::Time::now();
    Eigen::Vector3d body_p(odom_msg->pose.pose.position.x,
                           odom_msg->pose.pose.position.y,
                           odom_msg->pose.pose.position.z);
    Eigen::Quaterniond body_q(
        odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
    Eigen::Vector3d cam_p = body_q.toRotationMatrix() * cam2body_p_ + body_p;
    Eigen::Quaterniond cam_q = body_q * Eigen::Quaterniond(cam2body_R_);
    cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_msg);
    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      (depth_ptr->image).convertTo(depth_ptr->image, CV_16UC1, camConfig_.depth_scaling_factor);
    }
    cv::Mat depth_img = depth_ptr->image;

    // pub target

    int nr = depth_img.rows;
    int nc = depth_img.cols;
    std::vector<Eigen::Vector3d> obs_pts;
    // put the points of the depth into the list of obs_points

    // TODO depth filter

    // t1 = ros::Time::now();
    for (int i = depth_filter_margin_; i < nr - depth_filter_margin_; i += down_sample_factor_) {
      for (int j = depth_filter_margin_; j < nc - depth_filter_margin_; j += down_sample_factor_) {
        // (x,y,z) in camera frame
        double z = (depth_img.at<uint16_t>(i, j)) / camConfig_.depth_scaling_factor;
        if (depth_img.at<uint16_t>(i, j) == 0) {
          z = camConfig_.range + 0.5;
        }
        if (std::isnan(z) || std::isinf(z))
          continue;
        if (z < depth_filter_mindist_) {
          continue;
        }
        double y = (i - camConfig_.cy) * z / camConfig_.fy;
        double x = (j - camConfig_.cx) * z / camConfig_.fx;
        Eigen::Vector3d p(x, y, z);
        p = cam_q * p + cam_p;
        bool good_point = true;
        if (get_first_frame_) {
          // NOTE depth filter:
          Eigen::Vector3d p_rev_proj =
              last_cam_q_.inverse().toRotationMatrix() * (p - last_cam_p_);
          double vv = p_rev_proj.y() * camConfig_.fy / p_rev_proj.z() + camConfig_.cy;
          double uu = p_rev_proj.x() * camConfig_.fx / p_rev_proj.z() + camConfig_.cx;
          if (vv >= 0 && vv < nr && uu >= 0 && uu < nc) {
            double drift_dis = fabs(last_depth_.at<uint16_t>((int)vv, (int)uu) / camConfig_.depth_scaling_factor - p_rev_proj.z());
            if (drift_dis > depth_filter_tolerance_) {
              good_point = false;
            }
          }
        }
        if (good_point) {
          obs_pts.push_back(p);
        }
      }
    }
    last_depth_ = depth_img;
    last_cam_p_ = cam_p;
    last_cam_q_ = cam_q;
    get_first_frame_ = true;
    gridmap_.updateMap(cam_p, obs_pts);

    car_msgs::OccMap3d gridmap_msg;
    gridmap_msg.header.frame_id = "world";
    gridmap_msg.header.stamp = ros::Time::now();
    gridmap_.to_msg(gridmap_msg);
    gridmap_pub_.publish(gridmap_msg);

    OccGridMap gridmap_inflate = gridmap_;
    gridmap_inflate.inflate(inflate_size_);

    gridmap_inflate.to_msg(gridmap_msg);
    gridmap_inflate_pub_.publish(gridmap_msg);

  }

  void MapManager::cloud_odom_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                          const nav_msgs::OdometryConstPtr& odom_msg) {
    if (!map_recieved_) {
      map_recieved_ = true;
    }

    Eigen::Vector3d body_p(odom_msg->pose.pose.position.x,
                           odom_msg->pose.pose.position.y,
                           odom_msg->pose.pose.position.z);
    Eigen::Quaterniond body_q(
        odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
    Eigen::Vector3d sensor_p = body_q.toRotationMatrix() * lidar2body_p_ + body_p;
    // Eigen::Quaterniond sensor_q = body_q * Eigen::Quaterniond(lidar2body_R_);

    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    pcl::fromROSMsg(*cloud_msg, point_cloud);
    std::vector<Eigen::Vector3d> obs_pts;
    for(const auto& pts : point_cloud){
      Eigen::Vector3d p(pts.x, pts.y, pts.z);
      // @NOTE if the points are global data, otherwise they are in the lidar frame  
      // p = sensor_q * p + sensor_p;
      obs_pts.push_back(p);
    }
    gridmap_.updateMap(sensor_p, obs_pts);

    // publish map
    car_msgs::OccMap3d gridmap_msg;
    gridmap_msg.header.frame_id = "world";
    gridmap_msg.header.stamp = ros::Time::now();
    gridmap_.to_msg(gridmap_msg);
    gridmap_pub_.publish(gridmap_msg);

    OccGridMap gridmap_inflate = gridmap_;
    gridmap_inflate.inflate(inflate_size_);

    gridmap_inflate.to_msg(gridmap_msg);
    gridmap_inflate_pub_.publish(gridmap_msg);

  }

  void MapManager::init(ros::NodeHandle& nh) {
    // set parameters of mapping
    double res;
    std::string map_topic, inf_map_topic;
    
    nh.getParam("sensor_type", sensor_type_);
    if(!nh.getParam("map_topic", map_topic))
    {
      map_topic = "gridmap";
    }
    if(!nh.getParam("inf_map_topic", inf_map_topic))
    {
      inf_map_topic = "gridmap_inflate";
    }

    vehicle_param_.reset(new VehicleParam());
    gridmap_pub_ = nh.advertise<car_msgs::OccMap3d>(map_topic, 1);
    gridmap_inflate_pub_ = nh.advertise<car_msgs::OccMap3d>(inf_map_topic, 1);

    if (sensor_type_ == 0) {  // global map
      nh.getParam("resolution", res);
      nh.getParam("local_x", map_size_.x());
      nh.getParam("local_y", map_size_.y());
      // map_size_.z() = std::max(vehicle_param_->odom_to_ground(), vehicle_param_->odom_to_top()) + 3 * res;
      nh.getParam("local_z", map_size_.z());
      nh.getParam("origin_x", origin_.x());
      nh.getParam("origin_y", origin_.y());
      nh.getParam("origin_z", origin_.z());
      nh.getParam("inflate_size", inflate_size_);
      nh.getParam("prob_hit_log", prob_hit_log_);
      nh.getParam("prob_miss_log", prob_miss_log_);
      nh.getParam("clamp_min_log", clamp_min_log_);
      nh.getParam("clamp_max_log", clamp_max_log_);
      nh.getParam("min_occupancy_log", min_occupancy_log_);
      nh.getParam("min_ray_length", min_ray_length_);
      nh.getParam("max_ray_length", max_ray_length_);
      resolution_ = res;
      resolution_inv_ = 1 / resolution_;
      for(int i=0;i<3;i++) 
      {
          global_map_size_(i) = ceil(map_size_(i) / resolution_);
      }
      //inititalize size of buffer
      grid_size_y_multiply_z_ = global_map_size_(1) * global_map_size_(2);
      buffer_size_ = global_map_size_(0) * grid_size_y_multiply_z_; //The size of the global map
      buffer_size_2d_ = global_map_size_(0) * global_map_size_(1);
      occupancy_buffer_.resize(buffer_size_);
      occupancy_buffer_2d_.resize(buffer_size_2d_);

      cache_all_.resize(buffer_size_);
      cache_hit_.resize(buffer_size_);
      cache_rayend_.resize(buffer_size_);
      cache_traverse_.resize(buffer_size_);
      raycast_num_ = 0;

      fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), -1.0);
      fill(occupancy_buffer_2d_.begin(), occupancy_buffer_2d_.end(), -1.0);
      fill(cache_all_.begin(), cache_all_.end(), 0);
      fill(cache_hit_.begin(), cache_hit_.end(), 0);
      fill(cache_rayend_.begin(), cache_rayend_.end(), -1);
      fill(cache_traverse_.begin(), cache_traverse_.end(), -1);


      gridmap_.setup(res, map_size_, 10);  // sensor_range (10) here is no use, raycast only

    } else if(sensor_type_ == 1){  // depth camera, haven't test yet
      // camera parameters
      nh.getParam("camera_rate", camConfig_.rate);
      nh.getParam("camera_range", camConfig_.range);
      nh.getParam("cam_width", camConfig_.width);
      nh.getParam("cam_height", camConfig_.height);
      nh.getParam("cam_fx", camConfig_.fx);
      nh.getParam("cam_fy", camConfig_.fy);
      nh.getParam("cam_cx", camConfig_.cx);
      nh.getParam("cam_cy", camConfig_.cy);
      nh.getParam("depth_scaling_factor", camConfig_.depth_scaling_factor);
      // mapping parameters
      nh.getParam("down_sample_factor", down_sample_factor_);
      nh.getParam("resolution", res);
      nh.getParam("local_x", map_size_.x());
      nh.getParam("local_y", map_size_.y());
      nh.getParam("local_z", map_size_.z());
      nh.getParam("inflate_size", inflate_size_);
      gridmap_.setup(res, map_size_, camConfig_.range);
      // depth filter parameters
      nh.getParam("depth_filter_tolerance", depth_filter_tolerance_);
      nh.getParam("depth_filter_mindist", depth_filter_mindist_);
      nh.getParam("depth_filter_margin", depth_filter_margin_);
      // raycasting parameters
      int p_min, p_max, p_hit, p_mis, p_occ, p_def;
      nh.getParam("p_min", p_min);
      nh.getParam("p_max", p_max);
      nh.getParam("p_hit", p_hit);
      nh.getParam("p_mis", p_mis);
      nh.getParam("p_occ", p_occ);
      nh.getParam("p_def", p_def);
      gridmap_.setupP(p_min, p_max, p_hit, p_mis, p_occ, p_def);

      // camera extrinsics
      std::vector<double> tmp;
      if (nh.param<std::vector<double>>("cam2body_R", tmp, std::vector<double>())) {
        cam2body_R_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(tmp.data(), 3, 3);
      }
      if (nh.param<std::vector<double>>("cam2body_p", tmp, std::vector<double>())) {
        cam2body_p_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(tmp.data(), 3, 1);
      }
    }
    else if(sensor_type_ == 2){  // lidar
      // mapping parameters
      double range;
      nh.getParam("resolution", res);
      nh.getParam("local_x", map_size_.x());
      nh.getParam("local_y", map_size_.y());
      nh.getParam("local_z", map_size_.z());
      nh.getParam("inflate_size", inflate_size_);
      nh.getParam("lidar_range", range);
      gridmap_.setup(res, map_size_, range);
      // raycasting parameters
      int p_min, p_max, p_hit, p_mis, p_occ, p_def;
      nh.getParam("p_min", p_min);
      nh.getParam("p_max", p_max);
      nh.getParam("p_hit", p_hit);
      nh.getParam("p_mis", p_mis);
      nh.getParam("p_occ", p_occ);
      nh.getParam("p_def", p_def);
      gridmap_.setupP(p_min, p_max, p_hit, p_mis, p_occ, p_def);

      // lidar extrinsics
      std::vector<double> tmp;
      if (nh.param<std::vector<double>>("lidar2body_R", tmp, std::vector<double>())) {
        lidar2body_R_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(tmp.data(), 3, 3);
      }
      if (nh.param<std::vector<double>>("lidar2body_p", tmp, std::vector<double>())) {
        lidar2body_p_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(tmp.data(), 3, 1);
      }
    }
    else if(sensor_type_ == 3){  // global from pcl
      nh.getParam("resolution", res);
      nh.getParam("x_length", map_size_.x());
      nh.getParam("y_length", map_size_.y());
      nh.getParam("z_length", map_size_.z());
      nh.getParam("inflate_size", inflate_size_);
      gridmap_.setup(res, map_size_, 10, true);  // sensor_range (10) here is no use
      nh.getParam("map_path", map_path_);
    }

    if (sensor_type_ == 0) {
      // map_pc_sub_ = nh.subscribe("sim_cloud", 10, &MapManager::map_call_back, this);
      // global_map_timer_ = nh.createTimer(ros::Duration(1.0), &MapManager::global_map_timer_callback, this);

      pc_sub_.subscribe(nh, "map", 1);
      odom_sub_.subscribe(nh, "odom", 50);
      // cloud_odom_sync_Ptr_ = std::make_shared<CloudOdomSynchronizer>(CloudOdomSyncPolicy(100), pc_sub_, odom_sub_);
      // cloud_odom_sync_Ptr_->registerCallback(boost::bind(&MapManager::map_odom_callback, this, _1, _2));
      
      history_view_cloud_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

      Local_Pointcloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "map", 20));
      Odometry_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "odom", 100));
      pointcloud_odom_sync_.reset(new message_filters::Synchronizer<SyncPolicyPointcloud_Odom>(SyncPolicyPointcloud_Odom(100), *Local_Pointcloud_sub_, *Odometry_sub_));
      pointcloud_odom_sync_->registerCallback(boost::bind(&MapManager::OdometryAndPointcloud_cb, this, _1, _2));

      global_view_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("mapping/global_view_cloud", 1);
      global_occ_vis_timer_ = nh.createTimer(ros::Duration(0.3), &MapManager::globalOccVis_cb, this);

    } else if(sensor_type_ == 1) {
      depth_sub_.subscribe(nh, "depth", 1);
      odom_sub_.subscribe(nh, "odom", 50);
      depth_odom_sync_Ptr_ = std::make_shared<ImageOdomSynchronizer>(ImageOdomSyncPolicy(100), depth_sub_, odom_sub_);
      depth_odom_sync_Ptr_->registerCallback(boost::bind(&MapManager::depth_odom_callback, this, _1, _2));
    }
    else if(sensor_type_ == 2){
      pc_sub_.subscribe(nh, "pc_map", 1);
      odom_sub_.subscribe(nh, "odom", 50);
      cloud_odom_sync_Ptr_ = std::make_shared<CloudOdomSynchronizer>(CloudOdomSyncPolicy(100), pc_sub_, odom_sub_);
      cloud_odom_sync_Ptr_->registerCallback(boost::bind(&MapManager::cloud_odom_callback, this, _1, _2));
    }
    else if(sensor_type_ == 3){
      pcl::PointCloud<pcl::PointXYZ> cloud;
      if(pcl::io::loadPCDFile<pcl::PointXYZ>(map_path_, cloud) == -1)
      {
          ROS_ERROR("[map manager]: read pcl map failed!");
      }
      else
      {
        // std::cout << "[map]: Loaded " << cloud.width * cloud.height << " points from" << map_path_ << std::endl;
        Eigen::Vector3d min_pt, max_pt;
        for (const auto& pt : cloud) {
          Eigen::Vector3d p(pt.x, pt.y, pt.z);
          if((-map_size_.x() < p.x()) && (p.x() < map_size_.x()) && (-map_size_.y() < p.y() ) && (p.y() < map_size_.y()) && (-map_size_.z() < p.z() ) && (p.z() < map_size_.z())) {
            gridmap_.setOcc(p);
          }
          min_pt = min_pt.cwiseMin(p);
          max_pt = max_pt.cwiseMax(p);
        }
        std::cout << "min xyz:" << min_pt.transpose() << " max xyz:" << max_pt.transpose() << std::endl;
        gridmap_.inflate(inflate_size_);
        map_recieved_ = true;
      }
      global_map_timer_ = nh.createTimer(ros::Duration(1.0), &MapManager::global_map_timer_callback, this);
    }
    else{
      ROS_ERROR("[Map]: sensor type error!");
    }

  }

  // @Changjia: mapping callback
  void MapManager::OdometryAndPointcloud_cb(const sensor_msgs::PointCloud2::ConstPtr &pcl_msg, const nav_msgs::Odometry::ConstPtr &odom_msg)  
  {
    map_recieved_ = true;
    // std::cout << "1111111111111111111111111111111111111111111111" << std::endl;
    sensor_msgs::PointCloud2 pcl_msg_out;
    Eigen::Quaterniond quaternion(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x, 
                                  odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
    Eigen::Matrix3d Rotation_matrix;
    Eigen::Vector3d Position_XYZ(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);

    Rotation_matrix = quaternion.toRotationMatrix();
    // center_position_ = Position_XYZ + Rotation_matrix * lidar2car_;
    center_position_ = Position_XYZ;
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudTransformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pcl_msg, laserCloudIn);
    Eigen::Vector3d LaserCloudIn_XYZ;
    Eigen::Vector3d LaserCloudTransformed_XYZ;
    number_of_points_ = laserCloudIn.points.size();
  
    for(int i=0; i<number_of_points_; i++)
    {
        LaserCloudIn_XYZ(0) = laserCloudIn.points[i].x;
        LaserCloudIn_XYZ(1) = laserCloudIn.points[i].y;
        LaserCloudIn_XYZ(2) = laserCloudIn.points[i].z;
        // LaserCloudTransformed_XYZ = Rotation_matrix*LaserCloudIn_XYZ + center_position_;
        // if(LaserCloudIn_XYZ(2) < - 3 * resolution_)
        //     continue;

        LaserCloudTransformed_XYZ = LaserCloudIn_XYZ;
        laserCloudTransformed->points.emplace_back(pcl::PointXYZ(LaserCloudTransformed_XYZ(0), LaserCloudTransformed_XYZ(1), LaserCloudTransformed_XYZ(2)));
    }
  
    number_of_points_ = laserCloudTransformed->points.size();
  
    raycastProcess(center_position_, laserCloudTransformed);  //center_position_ is the postion of the lidar

  
  }

  void MapManager::raycastProcess(const Eigen::Vector3d& t_wc, const pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudTransformed) //t_wc is the position of the lidar
  {
      if(number_of_points_ == 0)
          return;

      raycast_num_ += 1;
      int set_cache_idx;
      /*----------iterate over all points of a frame of pointcloud----------*/
      for(int i=0; i<number_of_points_; i++)  
      {
          Eigen::Vector3d pt_w(laserCloudTransformed->points[i].x, 
                              laserCloudTransformed->points[i].y, 
                              laserCloudTransformed->points[i].z);

          /*This part is used to elminate the pointcloud of other agents(or the tracking target)*/
          // bool inside_car = false;
          // if(have_received_freespaces_)
          // {
              
          //     for(int car = 0; car < cars_num_; car++)
          //     {
          //         if(car == car_id_)
          //             continue;

          //         Eigen::MatrixXd normalVec_and_points = vec_freespaces_[car];
          //         if((Eigen::Vector2d(pt_w.head(2)) - Eigen::Vector2d(normalVec_and_points.col(0).tail(2))).dot(normalVec_and_points.col(0).head(2)) <= 0
          //         && (Eigen::Vector2d(pt_w.head(2)) - Eigen::Vector2d(normalVec_and_points.col(1).tail(2))).dot(normalVec_and_points.col(1).head(2)) <= 0
          //         && (Eigen::Vector2d(pt_w.head(2)) - Eigen::Vector2d(normalVec_and_points.col(2).tail(2))).dot(normalVec_and_points.col(2).head(2)) <= 0
          //         && (Eigen::Vector2d(pt_w.head(2)) - Eigen::Vector2d(normalVec_and_points.col(3).tail(2))).dot(normalVec_and_points.col(3).head(2)) <= 0)
          //         {
          //             inside_car = true;
          //             break;
          //         }
                  
          //     }
              
          // }
          // if(inside_car)
          //     continue;
          /*******************************************************************************************/

          double length = (pt_w - t_wc).norm();
          if (length < min_ray_length_)
              continue;
          else if (length > max_ray_length_)
          {
              pt_w = (pt_w - t_wc) / length * max_ray_length_ + t_wc;
              set_cache_idx = setCacheOccupancy(pt_w, 0);
          }
          // else if(!isInMap(pt_w))
          // {
          //     pt_w = closetPointInMap(pt_w, t_wc);
          //     set_cache_idx = setCacheOccupancy(pt_w, 0);           
          // }
          else
              set_cache_idx = setCacheOccupancy(pt_w, 1);
          // if(!isInMap(pt_w) && length < 2.0)
          // {
          //     pt_w = closetPointInMap(pt_w, t_wc);
          //     set_cache_idx = setCacheOccupancy(pt_w, 0);
          // }
          // else
          //     set_cache_idx = setCacheOccupancy(pt_w, 1);

          if(set_cache_idx != INVALID_IDX)
          {
              if(cache_rayend_[set_cache_idx] == raycast_num_)
              {
                  continue;
              }
              else   
                  cache_rayend_[set_cache_idx] = raycast_num_;
          }

          RayCaster raycaster;
          bool need_ray = raycaster.setInput( pt_w / resolution_, t_wc / resolution_);
          if(!need_ray)
              continue;
          Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
          Eigen::Vector3d ray_pt;
          if(!raycaster.step(ray_pt))
              continue;
          while(raycaster.step(ray_pt))
          {
              Eigen::Vector3d tmp = (ray_pt + half) * resolution_;
              set_cache_idx = setCacheOccupancy(tmp, 0);
              if(set_cache_idx != INVALID_IDX)
              {
                  if(cache_traverse_[set_cache_idx] == raycast_num_)
                      break;
                  else
                      cache_traverse_[set_cache_idx] = raycast_num_;
              }
              // else
              //   break;
          }
      }

      while(!cache_voxel_.empty())
      {
          Eigen::Vector3i idx = cache_voxel_.front();
          int idx_ctns = idx(0) * grid_size_y_multiply_z_ + idx(1) * global_map_size_(2) + idx(2);
          cache_voxel_.pop();

          double log_odds_update =
              cache_hit_[idx_ctns] >= cache_all_[idx_ctns] - cache_hit_[idx_ctns] ? prob_hit_log_ : prob_miss_log_;
          cache_hit_[idx_ctns] = cache_all_[idx_ctns] = 0;

          if ((log_odds_update >= 0 && occupancy_buffer_[idx_ctns] >= clamp_max_log_) ||
              (log_odds_update <= 0 && occupancy_buffer_[idx_ctns] <= clamp_min_log_))
              continue;

              occupancy_buffer_[idx_ctns] =
                  std::min(std::max(occupancy_buffer_[idx_ctns] + log_odds_update, clamp_min_log_), clamp_max_log_);
                  // std::min(occupancy_buffer_[idx_ctns] + log_odds_update, clamp_max_log_);

              int number_of_occspaces_in_z = 0;
              // for(int z = filter_idx_; z < global_map_size_(2); z++)
              for(int z = 0; z < global_map_size_(2); z++)
              {
                  if(z > 1.5 / resolution_)
                    break;
                  int idx_ctns_3d = idx(0) * grid_size_y_multiply_z_ + idx(1) * global_map_size_(2) + z;
                  if(occupancy_buffer_[idx_ctns_3d] > min_occupancy_log_)
                      number_of_occspaces_in_z++;
              }
              int idx_ctns_2d = idx(1) * global_map_size_(0) + idx(0);
              if(number_of_occspaces_in_z >= 2)
                  occupancy_buffer_2d_[idx_ctns_2d] = 1;
              else
                  occupancy_buffer_2d_[idx_ctns_2d] = 0;


              // if(occupancy_buffer_[idx_ctns] > min_occupancy_log_)
              // {
              //     int idx_ctns_2d = idx(1) * global_map_size_(0) + idx(0);
              //     occupancy_buffer_2d_[idx_ctns_2d] = 1;
              // }
              // else
              // {
              //     int number_of_freespaces_in_z = 0;
              //     for(int z = 0; z < global_map_size_(2); z++)
              //     {
              //         int idx_ctns_3d = idx(0) * grid_size_y_multiply_z_ + idx(1) * global_map_size_(2) + z;
              //         if(occupancy_buffer_[idx_ctns_3d] < min_occupancy_log_)
              //         {
              //             number_of_freespaces_in_z++;
              //         }
              //         else
              //         {
              //             break;
              //         }
              //     }
              //     if(number_of_freespaces_in_z == global_map_size_(2))
              //     {
              //         int idx_ctns_2d = idx(1) * global_map_size_(0) + idx(0);
              //         occupancy_buffer_2d_[idx_ctns_2d] = 0;
              //     }
              // }

      }
  }

  int MapManager::setCacheOccupancy(const Eigen::Vector3d& pos, int occ)
  {
      if (occ != 1 && occ != 0)
      {
          return INVALID_IDX;
      }

      Eigen::Vector3i id;
      posToIndex(pos, id);

      if (!isInMap(id))
      {
          return INVALID_IDX;
      }

      int idx_ctns = id(0) * grid_size_y_multiply_z_ + id(1) * global_map_size_(2) + id(2);

      cache_all_[idx_ctns] += 1;

      if (cache_all_[idx_ctns] == 1)
      {
          cache_voxel_.push(id);
      }

      if (occ == 1)
          cache_hit_[idx_ctns] += 1;

      return idx_ctns;    
  }

  inline bool MapManager::isInMap(const Eigen::Vector3d &pos)
  {
      Eigen::Vector3i idx;
      posToIndex(pos, idx);
      return isInMap(idx);
  }

  inline bool MapManager::isInMap(const Eigen::Vector3i &id)
  {
      return ((id[0] | (global_map_size_[0] - 1 - id[0]) | id[1] | (global_map_size_[1] - 1 - id[1]) | id[2]| (global_map_size_[2] - 1 - id[2])) >= 0);
  };

  bool MapManager::isInMap2d(const Eigen::Vector2d &pos)
  {
      Eigen::Vector2i idx;
      posToIndex2d(pos, idx);
      return isInMap2d(idx);
  }

  bool MapManager::isInMap2d(const Eigen::Vector2i &id)
  {
      if(id(0) < 0 || id(0) >= global_map_size_(0) || id(1) < 0 || id(1) >= global_map_size_(1))
      {
          return false;
      }
      else
          return true;
  };

  void MapManager::posToIndex2d(const Eigen::Vector2d& pos, Eigen::Vector2i& id)
  {
      for(int i = 0; i < 2; i++)
          id(i) = floor((pos(i) - origin_(i)) * resolution_inv_);
  }

  void MapManager::indexToPos2d(const Eigen::Vector2i& id, Eigen::Vector2d& pos)
  {
      // pos = origin_;
      for(int i = 0; i < 2; i++)
          pos(i) = (id(i) + 0.5) * resolution_ + origin_(i);
  }

  void MapManager::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id)
  {
      for(int i = 0; i < 3; i++)
          id(i) = floor((pos(i) - origin_(i)) * resolution_inv_);
  }

  void MapManager::indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos)
  {
      pos = origin_;
      for(int i = 0; i < 3; i++)
          pos(i) = (id(i) + 0.5) * resolution_ + origin_(i);
  }

  int MapManager::getVoxelState2d(const Eigen::Vector2d &pos)
  {
      Eigen::Vector2i id;
      posToIndex2d(pos, id);
      if(!isInMap2d(id))
          return -1;
      // todo: add local map range

      return occupancy_buffer_2d_[id(1) * global_map_size_(0) + id(0)] > 0.5 ? 1 : 0;
  }


  // @zhiwei: 3d height collision
  bool MapManager::CheckCollisionUsingGlobalPosition(const Eigen::Vector2d &p)
  {
    // TODO height
    // bool collide = false;
    // Eigen::Vector3d p3d(p.x(), p.y(), 0.0);
    // for(double h = 0.0; h >= -0.3; h-=0.1)
    // {
    //   p3d(2)= h;
    //   if(gridmap_.isOccupied(p3d))
    //   {
    //     return true;
    //   }
    // }
    // return false;

    if(getVoxelState2d(p) == 1)
      return true;

    return false;
  }

  bool MapManager::CheckIfCollisionUsingPosAndYaw(const Eigen::Vector3d &state)
  {
    std::vector<Eigen::Vector2d> vertices;
    Eigen::Vector3d pose;  // cernter position
    Eigen::Vector2d shape;  // width, length

    pose(0) = state(0) + vehicle_param_->d_cr() * cos(state(2));   
    pose(1) = state(1) + vehicle_param_->d_cr() * sin(state(2));
    pose(2) = state(2);
    shape(0) = vehicle_param_->width();
    shape(1) = vehicle_param_->length();

    GetDenseVerticesOfOrientedBoundingBox(pose, shape, vertices);

    for (auto &v : vertices) {
      if(CheckCollisionUsingGlobalPosition(v)){
        return true;
      }
    }
    return false;
  }

  void MapManager::GetDenseVerticesOfOrientedBoundingBox(const Eigen::Vector3d &pose, const Eigen::Vector2d &shape, std::vector<Eigen::Vector2d> &vertices, double res)
  {
    vertices.clear();
    double cos_theta = cos(pose(2));
    double sin_theta = sin(pose(2));
    Eigen::Vector2d corner1(
        pose(0) + 0.5 * shape(1) * cos_theta + 0.5 * shape(0) * sin_theta,
        pose(1) + 0.5 * shape(1) * sin_theta - 0.5 * shape(0) * cos_theta);
    Eigen::Vector2d corner2(
        pose(0) + 0.5 * shape(1) * cos_theta - 0.5 * shape(0) * sin_theta,
        pose(1) + 0.5 * shape(1) * sin_theta + 0.5 * shape(0) * cos_theta);
    Eigen::Vector2d corner3(
        pose(0) - 0.5 * shape(1) * cos_theta - 0.5 * shape(0) * sin_theta,
        pose(1) - 0.5 * shape(1) * sin_theta + 0.5 * shape(0) * cos_theta);
    Eigen::Vector2d corner4(
        pose(0) - 0.5 * shape(1) * cos_theta + 0.5 * shape(0) * sin_theta,
        pose(1) - 0.5 * shape(1) * sin_theta - 0.5 * shape(0) * cos_theta);
    for(double dl = res;dl < (corner2-corner1).norm(); dl+=res){
      Eigen::Vector2d point12 = dl / (corner2-corner1).norm() * (corner2-corner1) + corner1;
      vertices.push_back(point12);
    }
    for(double dl = res;dl < (corner3-corner2).norm(); dl+=res){
      Eigen::Vector2d point23 = dl / (corner3-corner2).norm() * (corner3-corner2) + corner2;
      vertices.push_back(point23);
    }
    for(double dl = res;dl < (corner4-corner3).norm(); dl+=res){
      Eigen::Vector2d point34 = dl / (corner4-corner3).norm() * (corner4-corner3) + corner3;
      vertices.push_back(point34);
    }
    for(double dl = res;dl < (corner1-corner4).norm(); dl+=res){
      Eigen::Vector2d point41 = dl / (corner1-corner4).norm() * (corner1-corner4) + corner4;
      vertices.push_back(point41);
    }
    vertices.push_back(corner1);
    vertices.push_back(corner2);
    vertices.push_back(corner3);
    vertices.push_back(corner4);
  }
  
  bool MapManager::CheckIfCollisionUsingLine(const Eigen::Vector2d p1, const Eigen::Vector2d p2, double checkl)
  {
    for(double dl = 0.0; dl < (p2-p1).norm(); dl+=checkl){
      Eigen::Vector2d pos = (p2-p1)*dl/(p2-p1).norm()+p1;
      if(CheckCollisionUsingGlobalPosition(pos)){
        return true;
      }
    }
    return CheckCollisionUsingGlobalPosition(p2);
  }

  void MapManager::globalOccVis_cb(const ros::TimerEvent& e)
  {
      //for vis 3d
      // history_view_cloud_ptr_->points.clear();
      // for (int x = 0; x < global_map_size_[0]; ++x)
      //     for (int y = 0; y < global_map_size_[1]; ++y)
      //         for (int z = 0; z < global_map_size_[2]; ++z)
      //         {
      //         //cout << "p(): " << occupancy_buffer_[x * grid_size_y_multiply_z_ + y * grid_size_(2) + z] << endl;
      //             if (occupancy_buffer_[x * grid_size_y_multiply_z_ + y * global_map_size_(2) + z] > min_occupancy_log_)
      //             // if (occupancy_buffer_[x * grid_size_y_multiply_z_ + y * global_map_size_(2) + z] != -1 && occupancy_buffer_[x * grid_size_y_multiply_z_ + y * global_map_size_(2) + z] < min_occupancy_log_)
      //             // if (occupancy_buffer_[x * grid_size_y_multiply_z_ + y * global_map_size_(2) + z] == -1)
      //             {
      //                 Eigen::Vector3i idx(x,y,z);
      //                 Eigen::Vector3d pos;
      //                 indexToPos(idx, pos);
      //                 pcl::PointXYZ pc(pos[0], pos[1], pos[2]);
      //                 history_view_cloud_ptr_->points.push_back(pc);
      //             }
      //         }

      //for vis 2d
      history_view_cloud_ptr_->points.clear();
      // int z = 0.2;
      for (int x = 0; x < global_map_size_[0]; ++x)
          for (int y = 0; y < global_map_size_[1]; ++y)
          {
              if (occupancy_buffer_2d_.at(y * global_map_size_(0) + x) > 0.7)
              {
                  Eigen::Vector2i idx(x, y);
                  Eigen::Vector2d pos;
                  indexToPos2d(idx, pos);
                  pcl::PointXYZ pc(pos[0], pos[1], 0.2);
                  history_view_cloud_ptr_->points.push_back(pc);
              }
          }


      history_view_cloud_ptr_->width = 1;
      history_view_cloud_ptr_->height = history_view_cloud_ptr_->points.size();
      history_view_cloud_ptr_->is_dense = true;
      history_view_cloud_ptr_->header.frame_id = "world";
      sensor_msgs::PointCloud2 cloud_msg;
      pcl::toROSMsg(*history_view_cloud_ptr_, cloud_msg);
      global_view_cloud_pub_.publish(cloud_msg);    
  }



}  // namespace mapping
