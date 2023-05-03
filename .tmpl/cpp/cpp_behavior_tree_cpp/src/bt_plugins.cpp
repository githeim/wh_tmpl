#include <unistd.h>
#include "bt_plugins.h"

BT::NodeStatus CheckBattery()
{
  static int iCntBatteryLevel=0;
  if (iCntBatteryLevel < 5) {
    iCntBatteryLevel++;
    printf("\033[1;33m[%s][%d] :x: Step [Battery Charging] ; Level %d/5 \033[m\n",
        __FUNCTION__,__LINE__, iCntBatteryLevel);

    if (iCntBatteryLevel ==5 ) {
      printf("\033[1;32m[%s][%d] :x: Step Done [Battery Charging]\033[m\n",
          __FUNCTION__,__LINE__);
    }

    return BT::NodeStatus::RUNNING;
  }
  return BT::NodeStatus::SUCCESS;
}


void MoveAction::halt() 
{
// in the overriden halt() method, you can do some cleanup, but do not forget to
// invoke the base class method AsyncActionNode::halt();
  m_bHalt_requested.store(true);
  AsyncActionNode::halt();
}

BT::NodeStatus MoveAction::tick() {

  auto x = getInput<float>("x");
  if (!x)
  {
    throw BT::RuntimeError( "missing required input [x]: ", x.error() );
  }
  auto y = getInput<float>("y");
  if (!y)
  {
    throw BT::RuntimeError( "missing required input [y]: ", y.error() );
  }
  auto yaw = getInput<float>("yaw");
  if (!yaw)
  {
    throw BT::RuntimeError( "missing required input [yaw]: ", yaw.error() );
  }

  while (!isHaltRequested()) {
    usleep(300000);
    if (m_iMoveCount <4) {
      m_iMoveCount++;
      printf("\033[1;33m[%s][%d] :x: Step [Move to] (%f %f %f) %d / 4 \033[m\n",
          __FUNCTION__,__LINE__,x.value(),y.value(),yaw.value(),m_iMoveCount);
      if (m_iMoveCount == 4) {
        printf("\033[1;32m[%s][%d] :x: Step Done [Move to] (%f %f %f) \033[m\n",
            __FUNCTION__,__LINE__,x.value(),y.value(),yaw.value());
      }
    }else{
      break;
    }
  }

  return BT::NodeStatus::SUCCESS;
}



BT::NodeStatus OpenDoor::tick()
{
  auto Door=getInput<std::string>("Door");
  if (!Door) {
    throw BT::RuntimeError( "missing required input [Door]: ", Door.error() );
  }

  if (m_iTryCount < 3 ) {
    m_iTryCount++;
    printf("\033[1;33m[%s][%d] :x: Try [%s] Open : try count  %d \033[m\n",
        __FUNCTION__,__LINE__,Door.value().c_str(),m_iTryCount);
    if ( m_iTryCount == 3) {
      printf("\033[1;32m[%s][%d] :x: the [%s] is opened  \033[m\n",
          __FUNCTION__,__LINE__,Door.value().c_str());
    }

    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}
