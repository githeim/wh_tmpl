#include <functional>
#include <memory>
#include <thread>

#include "test_action_interface/action/test_mission.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include<cmath>

float GetDistanceFromZero(float fX,float fY,float fZ) {
  return sqrt(pow(fX,2)+pow(fY,2)+pow(fZ,2));
}

class TestMissionActionServer : public rclcpp::Node
{
public:
  using TestMission = test_action_interface::action::TestMission;
  using GoalHandleTestMission = rclcpp_action::ServerGoalHandle<TestMission>;

  explicit TestMissionActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("test_mission_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<TestMission>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "test_mission",
      std::bind(&TestMissionActionServer::handle_goal, this, _1, _2),
      std::bind(&TestMissionActionServer::handle_cancel, this, _1),
      std::bind(&TestMissionActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<TestMission>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const TestMission::Goal> goal)
  {
    (void)uuid;
    // reject action request if it is far from 100 Meters
    auto fDistance = GetDistanceFromZero(goal->pose.pose.position.x,
                      goal->pose.pose.position.y,
                      goal->pose.pose.position.z);
    RCLCPP_INFO(this->get_logger(), "Received goal pose (%f,%f,%f),dist %f", 
        goal->pose.pose.position.x,
        goal->pose.pose.position.y,
        goal->pose.pose.position.z,
        fDistance
        );

    if ( fDistance > 100.f) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "goal accepted " );
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTestMission> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void execute(const std::shared_ptr<GoalHandleTestMission> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    
    rclcpp::Rate loop_rate(10); // 10 Hz
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<TestMission::Feedback>();
    auto result = std::make_shared<TestMission::Result>();

    auto & completion_percentage = feedback->completion_percentage;
    auto & passed_goals = feedback->passed_goals;
    
    std::vector<std::string>  vecPassed_Goals;
    for (int i= 1 ; i < 101 ; i++) {
      if (goal_handle->is_canceling()) {
        result->result=false;
        RCLCPP_INFO(this->get_logger(), "Goal Canceled");
        return;
      }
      completion_percentage = i;
      RCLCPP_INFO(this->get_logger(), "completion_percentage : %d",completion_percentage);
      if (i % 10 ==0) {
        std::string strGoal= std::string("goal_")+std::to_string(i/10);
        vecPassed_Goals.push_back(strGoal);
        passed_goals = vecPassed_Goals;
      }
      goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
    }

    if (rclcpp::ok()) {
      result->result = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    }
  }

  void handle_accepted(const std::shared_ptr<GoalHandleTestMission> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&TestMissionActionServer::execute, this, _1), goal_handle}.detach();
  }
};  // class TestMissionActionServer

int main(int argc, char ** argv)
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto action_server = std::make_shared<TestMissionActionServer>();
  rclcpp::spin(action_server);
  rclcpp::shutdown();
  return 0;
}
