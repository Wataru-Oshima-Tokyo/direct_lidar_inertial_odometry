/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

#include "dlio/dlio.h"

// ROS
#include "rclcpp/rclcpp.hpp"
#include "dlio/srv/save_pcd.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

// PCL
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include "techshare_ros_pkg2/srv/get_pcd.hpp"

class dlio::MapNode: public rclcpp::Node {

public:

  MapNode();
  ~MapNode();

  void start();

private:

  void getParams();

  void publishTimer();
  void callbackKeyframe(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& keyframe);

  void savePCD(std::shared_ptr<dlio::srv::SavePCD::Request> req,
               std::shared_ptr<dlio::srv::SavePCD::Response> res);
  void getPCD(std::shared_ptr<techshare_ros_pkg2::srv::GetPCD::Request> req,
               std::shared_ptr<techshare_ros_pkg2::srv::GetPCD::Response> res);
  rclcpp::TimerBase::SharedPtr publish_timer;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_sub;
  rclcpp::CallbackGroup::SharedPtr keyframe_cb_group, save_pcd_cb_group;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr world_pub;

  rclcpp::Service<dlio::srv::SavePCD>::SharedPtr save_pcd_srv;
  rclcpp::Service<techshare_ros_pkg2::srv::GetPCD>::SharedPtr get_pcd_srv;

  pcl::PointCloud<PointType>::Ptr dlio_map;
  pcl::VoxelGrid<PointType> voxelgrid;

  std::string odom_frame;
  std::string save_map_path;

  double publish_freq_;
  double leaf_size_;

};
