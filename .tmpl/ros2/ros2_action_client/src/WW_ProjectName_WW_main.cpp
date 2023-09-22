#include <chrono>
#include <cinttypes>
#include <functional>
#include <future>
#include <memory>
#include <string>

#include "test_action_interface/action/test_mission.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class TestMissionActionClient : public rclcpp::Node
{
public:
  using TestMission = test_action_interface::action::TestMission;
  using GoalHandleTestMission = rclcpp_action::ClientGoalHandle<TestMission>;


  explicit TestMissionActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("test_mission_action_client", node_options), goal_done_(false)
  {
    this->client_ptr_ = rclcpp_action::create_client<TestMission>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "test_mission");
    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&TestMissionActionClient::send_goal, this));
  }

  bool is_goal_done() const
  {
    return this->goal_done_;
  }

  void send_goal()
  {
    using namespace std::placeholders;
printf("\033[1;33m[%s][%d] :x: chk \033[m\n",__FUNCTION__,__LINE__);

    this->timer_->cancel();

    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = TestMission::Goal();
    geometry_msgs::msg::PoseStamped Pose ;
    Pose.pose.position.x= 20;
    Pose.pose.position.y= 10;
    Pose.pose.position.y= 0;

    goal_msg.pose = Pose;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<TestMission>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&TestMissionActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&TestMissionActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&TestMissionActionClient::result_callback, this, _1);
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<TestMission>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;

  void goal_response_callback(GoalHandleTestMission::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleTestMission::SharedPtr,
    const std::shared_ptr<const TestMission::Feedback> feedback)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "completion_percentage: %d",
      feedback->completion_percentage);
  }

  void result_callback(const GoalHandleTestMission::WrappedResult & result)
  {
    this->goal_done_ = true;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Result received : %d",result.result->result);
    
  }
};  // class TestMissionActionClient

int main(int argc, char ** argv)
{
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<TestMissionActionClient>();

  while (!action_client->is_goal_done()) {
    rclcpp::spin_some(action_client);

  }
  rclcpp::shutdown();
  return 0;
}
