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

#include "dlio/map.h"

dlio::MapNode::MapNode(): Node("dlio_map_node") {

  this->getParams();

  this->publish_timer = this->create_wall_timer(std::chrono::duration<double>(this->publish_freq_), 
      std::bind(&dlio::MapNode::publishTimer, this));

  this->keyframe_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto keyframe_sub_opt = rclcpp::SubscriptionOptions();
  keyframe_sub_opt.callback_group = this->keyframe_cb_group;
  this->keyframe_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("keyframes", 10,
      std::bind(&dlio::MapNode::callbackKeyframe, this, std::placeholders::_1), keyframe_sub_opt);

  this->world_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("world_map", 100);

  this->save_pcd_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->save_pcd_srv = this->create_service<dlio::srv::SavePCD>("save_pcd",
      std::bind(&dlio::MapNode::savePCD, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, this->save_pcd_cb_group);
  this->get_pcd_srv = this->create_service<techshare_ros_pkg2::srv::GetPCD>("get_pcd",
      std::bind(&dlio::MapNode::getPCD, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, this->save_pcd_cb_group);

  this->dlio_map = std::make_shared<pcl::PointCloud<PointType>>();

  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

}

dlio::MapNode::~MapNode() {}

void dlio::MapNode::getParams() {

  this->declare_parameter<std::string>("frames/odom", "odom");
  this->declare_parameter<double>("map/sparse/frequency", 1.0);
  this->declare_parameter<double>("map/sparse/leafSize", 0.5);
  this->declare_parameter<std::string>("save_map_path", "/");

  this->get_parameter("frames/odom", this->odom_frame);
  this->get_parameter("map/sparse/frequency", this->publish_freq_);
  this->get_parameter("map/sparse/leafSize", this->leaf_size_);
  this->get_parameter("save_map_path", this->save_map_path);
}

void dlio::MapNode::start() {
}

void dlio::MapNode::publishTimer() {

  if (this->dlio_map->points.size() == this->dlio_map->width * this->dlio_map->height) {
    sensor_msgs::msg::PointCloud2 world_ros;
    pcl::toROSMsg(*this->dlio_map, world_ros);
    world_ros.header.stamp = this->now();
    world_ros.header.frame_id = this->odom_frame;
    this->world_pub->publish(world_ros);
  }

}

void dlio::MapNode::callbackKeyframe(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& keyframe) {

  // convert scan to pcl format
  pcl::PointCloud<PointType>::Ptr keyframe_pcl = std::make_shared<pcl::PointCloud<PointType>>();
  pcl::fromROSMsg(*keyframe, *keyframe_pcl);

  // voxel filter
  this->voxelgrid.setLeafSize(this->leaf_size_, this->leaf_size_, this->leaf_size_);
  this->voxelgrid.setInputCloud(keyframe_pcl);
  this->voxelgrid.filter(*keyframe_pcl);

  // save filtered keyframe to map for rviz
  *this->dlio_map += *keyframe_pcl;

}

void dlio::MapNode::getPCD(std::shared_ptr<techshare_ros_pkg2::srv::GetPCD::Request> req,
                            std::shared_ptr<techshare_ros_pkg2::srv::GetPCD::Response> res) {

  pcl::PointCloud<PointType>::Ptr m = std::make_shared<pcl::PointCloud<PointType>>(*this->dlio_map);
    // Convert the pcl::PointCloud to sensor_msgs::msg::PointCloud2
  float leaf_size = req->leaf_size;
    // voxelize map
  pcl::VoxelGrid<PointType> vg;
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.setInputCloud(m);
  vg.filter(*m);
  sensor_msgs::msg::PointCloud2 ros_cloud;
  pcl::toROSMsg(*m, ros_cloud);

  res->cloud = ros_cloud;
  res->success = true;
  res->message = "Succeeded to covner the cloud";
}

void dlio::MapNode::savePCD(std::shared_ptr<dlio::srv::SavePCD::Request> req,
                            std::shared_ptr<dlio::srv::SavePCD::Response> res) {

  pcl::PointCloud<PointType>::Ptr m = std::make_shared<pcl::PointCloud<PointType>>(*this->dlio_map);

  float leaf_size = req->leaf_size;
  std::string p;
  if (req->save_path.empty()) {
    p = this->save_map_path; //->save_path;
  } else {
    p = req->save_path;
  }
  
  std::cout << std::setprecision(2) << "Saving map to " << p + "/dlio_map.pcd"
    << " with leaf size " << to_string_with_precision(leaf_size, 2) << "... "; std::cout.flush();

  // voxelize map
  pcl::VoxelGrid<PointType> vg;
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.setInputCloud(m);
  vg.filter(*m);

  // save map
  int ret = pcl::io::savePCDFileBinary(p + "/dlio_map.pcd", *m);
  res->success = ret == 0;

  if (res->success) {
    std::cout << "done" << std::endl;
    res->message = "Map saved successfully.";
  } else {
    std::cout << "failed" << std::endl;
    res->message = "Failed to save the map.";
  }
}
