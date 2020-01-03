#include "WW_ProjectName_WW/WW_NodeClassName_WW.h"
using namespace rclcpp_lifecycle;


WW_NodeClassName_WW::WW_NodeClassName_WW(const std::string & node_name, bool intra_process_comms )
  : rclcpp_lifecycle::LifecycleNode(node_name,
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{
}

node_interfaces::LifecycleNodeInterface::CallbackReturn
WW_NodeClassName_WW::on_configure(const State &){
  RCLCPP_INFO(get_logger(), __FUNCTION__);

  std::string strTopicName = "WW_SubscribeTopicName_WW";

  m_pSub = this->create_subscription<std_msgs::msg::String>(strTopicName,10,
                     std::bind(&WW_NodeClassName_WW::Data_CallBack, 
                           this, std::placeholders::_1) );
  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
  void
WW_NodeClassName_WW::Data_CallBack(const std_msgs::msg::String::SharedPtr pMsg)
{
  RCLCPP_INFO(get_logger(), "data_callback: %s", pMsg->data.c_str());
}
node_interfaces::LifecycleNodeInterface::CallbackReturn
WW_NodeClassName_WW::on_activate(const State &){
  RCLCPP_INFO(get_logger(), __FUNCTION__);
  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn
WW_NodeClassName_WW::on_deactivate(const State &){
  RCLCPP_INFO(get_logger(), __FUNCTION__);
  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn
WW_NodeClassName_WW::on_cleanup(const State &){
  RCLCPP_INFO(get_logger(), __FUNCTION__);

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn
WW_NodeClassName_WW::on_shutdown(const State & ){
  RCLCPP_INFO(get_logger(), __FUNCTION__);

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn
WW_NodeClassName_WW::on_error(const State & ){
  RCLCPP_INFO(get_logger(), __FUNCTION__);
  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
