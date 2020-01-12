#include "stdlib.h"
#include "WW_ProjectName_WW/WW_NodeClassName_WW.h"
int main(int argc, char *argv[])
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<WW_NodeClassName_WW> pNode =
    std::make_shared<WW_NodeClassName_WW>("WW_ProjectName_WW");

  exe.add_node(pNode->get_node_base_interface());
  exe.spin();

  rclcpp::shutdown();


  return 0;
}
