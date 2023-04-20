#include "WW_ProjectName_WW/WW_NodeClassName_WW.h"
using namespace rclcpp_lifecycle;


WW_NodeClassName_WW::WW_NodeClassName_WW(const std::string & node_name, bool intra_process_comms )
  : rclcpp_lifecycle::LifecycleNode(node_name,
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)),
  m_fEnv_Val_0(this->declare_parameter<float>("env_val_0")),
  m_fEnv_Val_1(this->declare_parameter<float>("env_val_1")),
  m_fEnv_Val_2(this->declare_parameter<float>("env_val_2")),
  m_fEnv_Val_3(this->declare_parameter<float>("env_val_3"))
{
  RCLCPP_INFO(get_logger(), "env_val_0: %f", this->m_fEnv_Val_0);
  RCLCPP_INFO(get_logger(), "env_val_1: %f", this->m_fEnv_Val_1);
  RCLCPP_INFO(get_logger(), "env_val_2: %f", this->m_fEnv_Val_2);
  RCLCPP_INFO(get_logger(), "env_val_3: %f", this->m_fEnv_Val_3);
}

node_interfaces::LifecycleNodeInterface::CallbackReturn
WW_NodeClassName_WW::on_configure(const State &){
  RCLCPP_INFO(get_logger(), __FUNCTION__);

  std::string strTopicName = "WW_PublishTopicName_WW";
  m_pPub = this->create_publisher<std_msgs::msg::String>(strTopicName, 10);
  m_pTimer = this->create_wall_timer(
      1s, std::bind(&WW_NodeClassName_WW::publish, this));


  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn
WW_NodeClassName_WW::on_activate(const State &){
  RCLCPP_INFO(get_logger(), __FUNCTION__);
  m_pPub->on_activate();
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
  m_pTimer.reset();
  m_pPub.reset();

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn
WW_NodeClassName_WW::on_shutdown(const State & ){
  RCLCPP_INFO(get_logger(), __FUNCTION__);
  m_pTimer.reset();
  m_pPub.reset();

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn
WW_NodeClassName_WW::on_error(const State & ){
  RCLCPP_INFO(get_logger(), __FUNCTION__);
  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void 
WW_NodeClassName_WW::publish() {

  printf("\033[1;33m[%s][%d] :x: publish \033[m\n",__FUNCTION__,__LINE__);
  static size_t count = 0;
  auto msg = std::make_unique<std_msgs::msg::String>();
  std::string strMsgString = "WW_Publish_Message_WW";
  msg->data = strMsgString+ " " + std::to_string(++count);

  // Print the current state for demo purposes
  if (!m_pPub->is_activated()) {
    RCLCPP_INFO(
      get_logger(), "Lifecycle publisher is currently inactive."
                    " Messages are not published.");
  } else {
    RCLCPP_INFO(
      get_logger(), "Lifecycle publisher is active."
                    " Publishing: [%s]", msg->data.c_str());
  }
  m_pPub->publish(std::move(msg));

  return ;
}

