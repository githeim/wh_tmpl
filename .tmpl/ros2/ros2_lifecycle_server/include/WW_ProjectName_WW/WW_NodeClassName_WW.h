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

#include <inttypes.h>
#include "test_srv_interface/srv/id_name_chk.hpp"

using namespace std::chrono_literals;

using namespace rclcpp_lifecycle;
using IdNameChk = test_srv_interface::srv::IdNameChk;

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

private:
  void svc_handler( const std::shared_ptr<rmw_request_id_t> request_header,
                    const std::shared_ptr<IdNameChk::Request> request,
                    const std::shared_ptr<IdNameChk::Response> response);

  rclcpp::Service<IdNameChk>::SharedPtr m_pServerInstance;
};


#endif // _WW_NodeClassName_WW_H_
