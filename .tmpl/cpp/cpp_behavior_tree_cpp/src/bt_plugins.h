#ifndef _BT_PLUGINS_H_
#define _BT_PLUGINS_H_
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"




BT::NodeStatus CheckBattery();


class OpenDoor : public BT::SyncActionNode {
public:
  OpenDoor(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode(name,config) {
    m_iTryCount= 0;
  }

  BT::NodeStatus tick() override;
  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    return{ 
      BT::InputPort<std::string>("Door")
       };
  }



  int tickCount() const
  {
    return m_iTryCount;
  }

  void resetTicks()
  {
    m_iTryCount = 0;
  }

private:
  int m_iTryCount;

};

class MoveAction : public BT::AsyncActionNode {
public:
  MoveAction(const std::string& name, const BT::NodeConfiguration& config) :
    AsyncActionNode(name,config){
      m_iMoveCount = 0;
    }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    return{ BT::InputPort<float>("x"),
      BT::InputPort<float>("y"),
      BT::InputPort<float>("yaw") };
  }

  BT::NodeStatus tick() override;

  virtual void halt() override;

private:
  std::atomic_bool m_bHalt_requested;
  int m_iMoveCount;


};

#endif /* ifndef _BT_PLUGINS_H_ */
