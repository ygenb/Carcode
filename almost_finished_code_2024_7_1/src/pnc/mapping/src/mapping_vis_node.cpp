#include <mapping/mapping.h>
#include <car_msgs/OccMap3d.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vehicle_params/vehicle_param.h>

ros::Publisher gridmap_vs_pub, gridmap2D_vs_pub, gridmap_inflate_vs_pub, gridmap2D_inflate_vs_pub;
VehicleParam params;

void gridmap_callback(const car_msgs::OccMap3dConstPtr& msgPtr) {
  mapping::OccGridMap gridmap;
  gridmap.from_msg(*msgPtr);
  sensor_msgs::PointCloud2 pc;
  gridmap.occ2pc(pc);
  pc.header.frame_id = "world";
  gridmap_vs_pub.publish(pc);
  // 2D
  gridmap.occ2pc2D(pc);
  gridmap2D_vs_pub.publish(pc);
}

void gridmap_inflate_callback(const car_msgs::OccMap3dConstPtr& msgPtr) {
  mapping::OccGridMap gridmap;
  gridmap.from_msg(*msgPtr);
  sensor_msgs::PointCloud2 pc;
  gridmap.occ2pc(pc);
  pc.header.frame_id = "world";
  gridmap_inflate_vs_pub.publish(pc);
  // 2D
  gridmap.occ2pc2D(pc);
  gridmap2D_inflate_vs_pub.publish(pc);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mapping_vis");
  ros::NodeHandle nh("~");
  gridmap_vs_pub = nh.advertise<sensor_msgs::PointCloud2>("vs_gridmap", 1);
  gridmap2D_vs_pub = nh.advertise<sensor_msgs::PointCloud2>("vs_gridmap2D", 1);
  gridmap_inflate_vs_pub = nh.advertise<sensor_msgs::PointCloud2>("vs_gridmap_inflate", 1);
  gridmap2D_inflate_vs_pub = nh.advertise<sensor_msgs::PointCloud2>("vs_gridmap2D_inflate", 1);
  ros::Subscriber gridmap_sub = nh.subscribe<car_msgs::OccMap3d>("gridmap", 1, gridmap_callback);
  ros::Subscriber gridmap_inflate_sub = nh.subscribe<car_msgs::OccMap3d>("gridmap_inflate", 1, gridmap_inflate_callback);

  ros::spin();
  return 0;
}
