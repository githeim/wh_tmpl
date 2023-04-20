#ifndef _WW_NodeClassName_WW_H_
#define _WW_NodeClassName_WW_H_

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <atomic>

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
  rclcpp::Client<IdNameChk>::SharedPtr m_pClientInstance;

  std::shared_ptr<std::thread> m_pMainLoopThread;
  std::atomic<bool> m_bMainLoopTrigger;
  bool Get_MainLoopTrigger();
  void Set_MainLoopTrigger(bool bVal);
  void MainLoop(rclcpp::Client<IdNameChk>::SharedPtr pCli); 
  float m_fEnv_Val_0;
  float m_fEnv_Val_1;
  float m_fEnv_Val_2;
  float m_fEnv_Val_3;

};

#endif // _WW_NodeClassName_WW_H_
