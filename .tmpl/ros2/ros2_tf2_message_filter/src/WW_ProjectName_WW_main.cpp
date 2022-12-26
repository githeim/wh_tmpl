#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "message_filters/subscriber.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class TF_Receiver : public rclcpp::Node
{
public:
  TF_Receiver()
  : Node("WW_ProjectName_WW")
  {
    // Declare and acquire `target_frame` parameter
    m_target_frame = this->declare_parameter<std::string>("target_frame", "odom");
    m_target_topic="/lidar/scan";
    RCLCPP_INFO(
        this->get_logger(), "m_target_frame : %s",m_target_frame.c_str());
    RCLCPP_INFO(
        this->get_logger(), "m_target_topic : %s",m_target_topic.c_str());


    std::chrono::duration<int> buffer_timeout(1);

    m_tf2_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // Create the timer interface before call to waitForTransform,
    // to avoid a tf2_ros::CreateTimerInterfaceException exception
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(),
      this->get_node_timers_interface());
    m_tf2_buffer->setCreateTimerInterface(timer_interface);

    // TransformListener object. Once the listener is created, 
    // it starts receiving tf2 transformations over the wire, 
    // and buffers them for up to 10 seconds.
    m_tf2_listener =
      std::make_shared<tf2_ros::TransformListener>(*m_tf2_buffer);

    // QOS Settings
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = 10;

    auto sub_opt = rclcpp::SubscriptionOptions();
    m_subScan = 
      std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan,rclcpp::Node>>
      (this,m_target_topic,custom_qos_profile,sub_opt);

    m_tf2_filter = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
        *m_subScan, *m_tf2_buffer, m_target_frame, 10,
        this->get_node_logging_interface(),
        this->get_node_clock_interface(),
        buffer_timeout);
    m_tf2_filter->registerCallback(&TF_Receiver::msgCallback, this);
  }

private:
  void msgCallback(const sensor_msgs::msg::LaserScan::SharedPtr point_ptr)
  {
    RCLCPP_INFO(
        this->get_logger(), "CB Call");
    (void)point_ptr;
  }
  std::string m_target_topic;
  std::string m_target_frame;
  std::shared_ptr<tf2_ros::Buffer> m_tf2_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tf2_listener;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan,rclcpp::Node>> m_subScan;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> m_tf2_filter;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TF_Receiver>());
  rclcpp::shutdown();
  return 0;
}
