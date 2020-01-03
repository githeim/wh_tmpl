#ifndef _WW_NodeClassName_WW_H_
#define _WW_NodeClassName_WW_H_

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "rcutils/logging_macros.h"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

using namespace rclcpp_lifecycle;


class WW_NodeClassName_WW : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit WW_NodeClassName_WW(
      const std::string & node_name, bool intra_process_comms = false);

  node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &);

  node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &);

  node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &);

  node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &);

  node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state);

  node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State & state);

  
  void Data_CallBack(const std_msgs::msg::String::SharedPtr pMsg);

  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> m_pSub;

  void publish();
private:

};


#endif // _WW_NodeClassName_WW_H_
