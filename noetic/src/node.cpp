#include "configuration.h"
#include "message_conversion.h"
#include <fast_lio/mapper.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <iostream>
#include <livox_ros_driver/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/UInt64.h>
#include <tf2_ros/transform_broadcaster.h>

class FastLioNode {
  using CloudMsg = sensor_msgs::PointCloud2;
  using OdomMsg = nav_msgs::Odometry;

public:
  FastLioNode()
      : private_("~"), config_(fast_lio::adapter::readConfig([this](const char *name, auto &value) {
          std::string key = name;
          std::replace(key.begin(), key.end(), '.', '/');
          if (!private_.getParam(key, value))
            node_.param(key, value, value);
        })),
        mapper_(config_.core, [this](fast_lio::OdometryKind kind, const fast_lio::Odometry &state) {
          publishOdometry(kind, state);
        }) {
    world_ = node_.advertise<CloudMsg>("/cloud_registered", 100000);
    body_ = node_.advertise<CloudMsg>("/cloud_registered_body", 100000);
    effect_ = node_.advertise<CloudMsg>("/cloud_effected", 100000);
    map_ = node_.advertise<CloudMsg>("/Laser_map", 100000);
    odom_ = node_.advertise<OdomMsg>("/Odometry", 100000);
    imu_odom_ = node_.advertise<OdomMsg>("/Odom_imu_state", 1000);
    gravity_ = node_.advertise<geometry_msgs::Vector3Stamped>(config_.gravity_topic, 100);
    path_ = node_.advertise<nav_msgs::Path>("/path", 100000);
    if (!config_.replay_lidar_ack_topic.empty())
      ack_ = node_.advertise<std_msgs::UInt64>(config_.replay_lidar_ack_topic, 10);
    if (config_.core.lidar_type == 1) {
      cloud_sub_ = node_.subscribe<livox_ros_driver::CustomMsg>(
          config_.lid_topic, 200000, [this](const livox_ros_driver::CustomMsg::ConstPtr &message) {
            if (fast_lio::adapter::livoxScan(*message, livox_scan_))
              mapper_.pushLivox(seconds(message->header.stamp), livox_scan_);
            drain();
          });
    } else {
      cloud_sub_ = node_.subscribe<CloudMsg>(
          config_.lid_topic, 200000, [this](const CloudMsg::ConstPtr &message) {
            pcl::PCLPointCloud2 cloud;
            pcl_conversions::toPCL(*message, cloud);
            mapper_.pushPointCloud(seconds(message->header.stamp), cloud);
            drain();
          });
    }
    imu_sub_ = node_.subscribe<sensor_msgs::Imu>(
        config_.imu_topic, 200000, [this](const sensor_msgs::Imu::ConstPtr &message) {
          mapper_.pushImu(fast_lio::adapter::imuSample(*message, message->header.stamp.toNSec()));
          drain();
        });
  }
  void finish() { mapper_.finish(); }

private:
  static double seconds(const ros::Time &time) { return time.toNSec() / 1e9; }
  void publishOdometry(fast_lio::OdometryKind kind, const fast_lio::Odometry &state) {
    OdomMsg message;
    message.header.stamp.fromNSec(state.stamp_ns);
    message.header.frame_id = config_.world_frame;
    message.child_frame_id = config_.body_frame;
    fast_lio::adapter::odometry(state, message);
    if (kind == fast_lio::OdometryKind::Imu) {
      imu_odom_.publish(message);
      return;
    }
    odom_.publish(message);
    geometry_msgs::TransformStamped transform;
    transform.header = message.header;
    transform.child_frame_id = message.child_frame_id;
    transform.transform.translation.x = state.position.x();
    transform.transform.translation.y = state.position.y();
    transform.transform.translation.z = state.position.z();
    transform.transform.rotation = message.pose.pose.orientation;
    broadcaster_.sendTransform(transform);
  }
  void drain() {
    for (;;) {
      auto output = mapper_.process();
      if (!output.consumed())
        return;
      if (!output.diagnostic.empty())
        ROS_INFO("%s", output.diagnostic.c_str());
      if (output.status == fast_lio::FrameStatus::Mapped)
        publish(output);
      if (ack_) {
        std_msgs::UInt64 message;
        message.data = ++sequence_;
        ack_.publish(message);
      }
    }
  }
  void publish(const fast_lio::FrameOutput &output) {
    std_msgs::Header header;
    header.stamp.fromNSec(output.odometry.stamp_ns);
    header.frame_id = config_.world_frame;
    geometry_msgs::Vector3Stamped gravity;
    gravity.header = header;
    gravity.vector.x = output.gravity.x();
    gravity.vector.y = output.gravity.y();
    gravity.vector.z = output.gravity.z();
    gravity_.publish(gravity);
    if (output.append_path) {
      geometry_msgs::PoseStamped pose;
      pose.header = header;
      fast_lio::adapter::pose(output.odometry, pose.pose);
      path_message_.header = pose.header;
      path_message_.poses.push_back(pose);
      if (path_message_.poses.size() > 10000)
        path_message_.poses.erase(path_message_.poses.begin());
      path_.publish(path_message_);
    }
    if (output.world_cloud) {
      CloudMsg cloud;
      pcl::toROSMsg(*output.world_cloud, cloud);
      cloud.header = header;
      world_.publish(cloud);
    }
    if (output.body_cloud) {
      CloudMsg cloud;
      pcl::toROSMsg(*output.body_cloud, cloud);
      cloud.header = header;
      cloud.header.frame_id = config_.body_frame;
      body_.publish(cloud);
    }
  }
  ros::NodeHandle node_, private_;
  fast_lio::adapter::NodeConfig config_;
  fast_lio::Mapper mapper_;
  fast_lio::LivoxScan livox_scan_;
  tf2_ros::TransformBroadcaster broadcaster_;
  nav_msgs::Path path_message_;
  uint64_t sequence_ = 0;
  ros::Subscriber cloud_sub_, imu_sub_;
  ros::Publisher world_, body_, effect_, map_, odom_, imu_odom_, gravity_, path_, ack_;
};
int main(int argc, char **argv) {
  ros::init(argc, argv, "laserMapping");
  try {
    FastLioNode node;
    ros::spin();
    node.finish();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
