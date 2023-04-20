#include "WW_ProjectName_WW/WW_NodeClassName_WW.h"
#include <functional>
using namespace std::placeholders;

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

  m_pServerInstance = this->create_service<IdNameChk>("id_name_chk", 
      std::bind(&WW_NodeClassName_WW::svc_handler,this,_1,_2,_3)
      );
  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
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

void 
WW_NodeClassName_WW::svc_handler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<IdNameChk::Request> request,
    const std::shared_ptr<IdNameChk::Response> response)
{
  (void)request_header;
  printf("\033[1;32m[%s][%d] :x: request id [%" PRId64 "] name[%s]\033[m\n",
      __FUNCTION__,__LINE__,
      request->id_number, 
      request->person_name.c_str()
      );

  response->is_correct = true;
  response->return_id_number = 23;
}
