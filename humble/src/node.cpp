#include "configuration.h"
#include "message_conversion.h"
#include <fast_lio/mapper.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <iostream>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <tf2_ros/transform_broadcaster.h>

class FastLioNode : public rclcpp::Node {
  using CloudMsg = sensor_msgs::msg::PointCloud2;
  using ImuMsg = sensor_msgs::msg::Imu;
  using LivoxMsg = livox_ros_driver2::msg::CustomMsg;
  using OdomMsg = nav_msgs::msg::Odometry;

public:
  FastLioNode()
      : Node("laserMapping"),
        config_(fast_lio::adapter::readConfig([this](const char *name, auto &value) {
          using T = std::decay_t<decltype(value)>;
          declare_parameter<T>(name, value);
          get_parameter(name, value);
        })),
        mapper_(config_.core,
                [this](fast_lio::OdometryKind kind, const fast_lio::Odometry &state) {
                  publishOdometry(kind, state);
                }),
        broadcaster_(this) {
    world_ = create_publisher<CloudMsg>("/cloud_registered", 100);
    body_ = create_publisher<CloudMsg>("/cloud_registered_body", 100);
    effect_ = create_publisher<CloudMsg>("/cloud_effected", 100);
    map_ = create_publisher<CloudMsg>("/Laser_map", 100);
    odom_ = create_publisher<OdomMsg>("/Odometry", 100);
    imu_odom_ = create_publisher<OdomMsg>("/Odom_imu_state", 1000);
    gravity_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(config_.gravity_topic, 100);
    path_ = create_publisher<nav_msgs::msg::Path>("/path", 100);
    if (!config_.replay_lidar_ack_topic.empty())
      ack_ = create_publisher<std_msgs::msg::UInt64>(config_.replay_lidar_ack_topic, 10);
    if (config_.core.lidar_type == 1) {
      livox_ = create_subscription<LivoxMsg>(
          config_.lid_topic, 20, [this](LivoxMsg::ConstSharedPtr message) {
            if (fast_lio::adapter::livoxScan(*message, livox_scan_))
              mapper_.pushLivox(rclcpp::Time(message->header.stamp).seconds(), livox_scan_);
            drain();
          });
    } else {
      cloud_ = create_subscription<CloudMsg>(
          config_.lid_topic, 20, [this](CloudMsg::ConstSharedPtr message) {
            pcl::PCLPointCloud2 cloud;
            pcl_conversions::toPCL(*message, cloud);
            mapper_.pushPointCloud(rclcpp::Time(message->header.stamp).seconds(), cloud);
            drain();
          });
    }
    imu_ =
        create_subscription<ImuMsg>(config_.imu_topic, 200, [this](ImuMsg::ConstSharedPtr message) {
          mapper_.pushImu(fast_lio::adapter::imuSample(
              *message, rclcpp::Time(message->header.stamp).nanoseconds()));
          drain();
        });
  }
  void finish() { mapper_.finish(); }

private:
  void publishOdometry(fast_lio::OdometryKind kind, const fast_lio::Odometry &state) {
    OdomMsg message;
    message.header.stamp = rclcpp::Time(state.stamp_ns);
    message.header.frame_id = config_.world_frame;
    message.child_frame_id = config_.body_frame;
    fast_lio::adapter::odometry(state, message);
    if (kind == fast_lio::OdometryKind::Imu) {
      imu_odom_->publish(message);
      return;
    }
    odom_->publish(message);
    geometry_msgs::msg::TransformStamped transform;
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
        RCLCPP_INFO(get_logger(), "%s", output.diagnostic.c_str());
      if (output.status == fast_lio::FrameStatus::Mapped)
        publish(output);
      if (ack_) {
        std_msgs::msg::UInt64 message;
        message.data = ++sequence_;
        ack_->publish(message);
      }
    }
  }
  void publish(const fast_lio::FrameOutput &output) {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(output.odometry.stamp_ns);
    header.frame_id = config_.world_frame;
    geometry_msgs::msg::Vector3Stamped gravity;
    gravity.header = header;
    gravity.vector.x = output.gravity.x();
    gravity.vector.y = output.gravity.y();
    gravity.vector.z = output.gravity.z();
    gravity_->publish(gravity);
    if (output.append_path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = header;
      fast_lio::adapter::pose(output.odometry, pose.pose);
      path_message_.header = pose.header;
      path_message_.poses.push_back(pose);
      if (path_message_.poses.size() > 10000)
        path_message_.poses.erase(path_message_.poses.begin());
      path_->publish(path_message_);
    }
    if (output.world_cloud) {
      CloudMsg cloud;
      pcl::toROSMsg(*output.world_cloud, cloud);
      cloud.header = header;
      world_->publish(cloud);
    }
    if (output.body_cloud) {
      CloudMsg cloud;
      pcl::toROSMsg(*output.body_cloud, cloud);
      cloud.header = header;
      cloud.header.frame_id = config_.body_frame;
      body_->publish(cloud);
    }
  }
  fast_lio::adapter::NodeConfig config_;
  fast_lio::Mapper mapper_;
  fast_lio::LivoxScan livox_scan_;
  tf2_ros::TransformBroadcaster broadcaster_;
  nav_msgs::msg::Path path_message_;
  uint64_t sequence_ = 0;
  rclcpp::Subscription<LivoxMsg>::SharedPtr livox_;
  rclcpp::Subscription<CloudMsg>::SharedPtr cloud_;
  rclcpp::Subscription<ImuMsg>::SharedPtr imu_;
  rclcpp::Publisher<CloudMsg>::SharedPtr world_, body_, effect_, map_;
  rclcpp::Publisher<OdomMsg>::SharedPtr odom_, imu_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gravity_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr ack_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<FastLioNode>();
    rclcpp::spin(node);
    node->finish();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
