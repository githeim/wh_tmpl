#include "stdlib.h"
#include "WW_ProjectName_WW/WW_NodeClassName_WW.h"
#include <thread>

bool g_bStopSpin=false;                                               
void Set_bStopSpin(const bool& bVal) {g_bStopSpin=bVal;}              
bool Get_bStopSpin() {return  g_bStopSpin;}                           
                                                                      
void StopSpinChecker(rclcpp::executors::SingleThreadedExecutor& exe) {
  while(!Get_bStopSpin()) {                                           
    usleep(500000);                                                   
  }                                                                   
  // Delay for shutdown state transition                              
  usleep(500000);                                                     
  exe.cancel();                                                       
  return;                                                             
}                                                                     

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

  // Thread for stopping spin
  auto thrSpinChecker = std::thread(StopSpinChecker,std::ref(exe));

  exe.spin();

  thrSpinChecker.join();
  rclcpp::shutdown();


  return 0;
}
