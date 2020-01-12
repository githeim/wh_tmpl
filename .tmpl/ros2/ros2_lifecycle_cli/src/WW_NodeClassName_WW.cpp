#include "WW_ProjectName_WW/WW_NodeClassName_WW.h"
#include<unistd.h>
using namespace rclcpp_lifecycle;

WW_NodeClassName_WW::WW_NodeClassName_WW(const std::string & node_name, 
                                         bool intra_process_comms )
  : rclcpp_lifecycle::LifecycleNode(node_name,
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{
}

node_interfaces::LifecycleNodeInterface::CallbackReturn
WW_NodeClassName_WW::on_configure(const State &){
  RCLCPP_INFO(get_logger(), __FUNCTION__);

  m_pClientInstance = this->create_client<IdNameChk>("id_name_chk");

  while (!m_pClientInstance->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {

      RCLCPP_ERROR(get_logger(), "Err)interruption is detected");
      return node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_logger(), "wait service establishing");
  }
  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn
WW_NodeClassName_WW::on_activate(const State &){
  RCLCPP_INFO(get_logger(), __FUNCTION__);
  // Create Client loop thread
  Set_MainLoopTrigger(true); 
  m_pMainLoopThread = std::make_shared<std::thread>(
                 &WW_NodeClassName_WW::MainLoop,this,m_pClientInstance);

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
  Set_MainLoopTrigger(false); 
  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn
WW_NodeClassName_WW::on_error(const State & ){
  RCLCPP_INFO(get_logger(), __FUNCTION__);
  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void WW_NodeClassName_WW::Set_MainLoopTrigger(bool bVal) {
  m_bMainLoopTrigger = bVal;
  printf("\033[1;36m[%s][%d] :x: Main Loop Trigger [%d]\033[m\n",
      __FUNCTION__,__LINE__,bVal);

  sleep(1);
}
bool WW_NodeClassName_WW::Get_MainLoopTrigger() {
  return m_bMainLoopTrigger;
}

void WW_NodeClassName_WW::MainLoop(
    rclcpp::Client<IdNameChk>::SharedPtr pCli) {
  auto pRequest = std::make_shared<IdNameChk::Request>();
  pRequest->id_number =71;
  pRequest->person_name ="JohnDoe";
  while (Get_MainLoopTrigger()) {
    usleep (500000); // 0.5 sec
    auto future = pCli->async_send_request(pRequest);

    future.wait_for(0.1s);
    auto response = future.get();
    printf("\033[1;33m[%s][%d] :x: pResponse = %" PRId64" \033[m\n",
        __FUNCTION__,__LINE__,response->return_id_number);
  }
}
