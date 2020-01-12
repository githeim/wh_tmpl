#include "stdlib.h"
#include "WW_ProjectName_WW/WW_NodeClassName_WW.h"
#if 1 // :x: for test

int main(int argc, char *argv[])
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<WW_NodeClassName_WW> pNode =
    std::make_shared<WW_NodeClassName_WW>("WW_ProjectName_WW");

  printf("\033[1;31m[%s][%d] :x: chk %s \033[m\n",__FUNCTION__,__LINE__,pNode->get_name());

  exe.add_node(pNode->get_node_base_interface());
  exe.spin();

  rclcpp::shutdown();


  return 0;
}
#endif // :x: for test

#if 0 // :x: for test

#include <chrono>
#include <cinttypes>
#include <memory>
#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using IdNameChk = test_srv_interface::srv::IdNameChk;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("minimal_client");
  auto client = node->create_client<IdNameChk>("id_name_chk");
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
  }
  auto request = std::make_shared<IdNameChk::Request>();
  request->id_number =71;
  request->person_name ="JohnDoe";


  auto result_future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "service call failed :(");
    return 1;
  }
  auto result = result_future.get();
printf("\033[1;36m[%s][%d] :x: ""result of %" PRId64 " \033[m\n",__FUNCTION__,__LINE__,result->return_id_number);

  rclcpp::shutdown();
  return 0;
}
#endif // :x: for test

