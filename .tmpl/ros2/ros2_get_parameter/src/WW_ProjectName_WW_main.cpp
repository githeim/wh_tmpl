#include "stdlib.h"
#include <thread>
#include <memory>
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
int main(int argc, char *argv[])
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("WW_ProjectName_WW");
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node, "test_lifecycle_pub");
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      printf("\033[1;33m[%s][%d] :x: Interrupted while waiting for the service. Exiting. \033[m\n",__FUNCTION__,__LINE__);
      rclcpp::shutdown();
    }
    printf("\033[1;33m[%s][%d] :x: service not available, waiting again... \033[m\n",__FUNCTION__,__LINE__);
    sleep(1);
  }
  while ( true ) {
    printf("\033[1;33m[%s][%d] :x: loop \033[m\n",__FUNCTION__,__LINE__);
    auto parameters = parameters_client->get_parameters({"publish_cnt"});
    for (auto & parameter : parameters)
    {
      printf("\033[1;33m[%s][%d] :x: Parameter name: %s value %s \033[m\n",
          __FUNCTION__,__LINE__,
          parameter.get_name().c_str(),
          parameter.value_to_string().c_str()
          );
    }
    sleep(1);
  }
  return 0;

}
