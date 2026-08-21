#include "manual_return_planner/command_gate.h"

#include <mars_quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <string>

namespace mrp = manual_return_planner;

// CommandGate is the sole publisher of the controller's position command topic
// (`/quad_0/planning/pos_cmd`).  It subscribes to the two candidate sources:
//
//   normal_pos_cmd : traj_server (normal flight) output
//   return_pos_cmd : ManualReturnExecutor output
//
// and forwards exactly one of them according to the current CommandSource.
// Switching is driven by two std_srvs/Trigger services and never by the
// arrival of a command, so a stale normal stream cannot re-take control.
class CommandGateNode {
 public:
  CommandGateNode() : private_nh_("~") {
    private_nh_.param("normal_command_topic", normal_topic_,
                      std::string("/manual_return/normal_pos_cmd"));
    private_nh_.param("return_command_topic", return_topic_,
                      std::string("/manual_return/return_pos_cmd"));
    private_nh_.param("controller_command_topic", controller_topic_,
                      std::string("/quad_0/planning/pos_cmd"));
    private_nh_.param("control_source_topic", source_topic_,
                      std::string("/manual_return/control_source"));

    normal_sub_ = nh_.subscribe(normal_topic_, 50,
                                &CommandGateNode::normalCallback, this);
    return_sub_ = nh_.subscribe(return_topic_, 50,
                                &CommandGateNode::returnCallback, this);
    output_pub_ = nh_.advertise<mars_quadrotor_msgs::PositionCommand>(
        controller_topic_, 50);
    source_pub_ = nh_.advertise<std_msgs::String>(source_topic_, 1, true);

    set_return_srv_ = nh_.advertiseService(
        "/manual_return/gate/set_manual_return",
        &CommandGateNode::setManualReturn, this);
    set_normal_srv_ = nh_.advertiseService(
        "/manual_return/gate/set_normal",
        &CommandGateNode::setNormal, this);

    publishSource();
    ROS_INFO_STREAM("[CommandGate] normal='"
                    << normal_topic_ << "' return='" << return_topic_
                    << "' output='" << controller_topic_ << "'");
  }

 private:
  bool setManualReturn(std_srvs::Trigger::Request&,
                       std_srvs::Trigger::Response& response) {
    gate_.switchToManualReturn();
    publishSource();
    response.success = true;
    response.message = "command source switched to MANUAL_RETURN";
    ROS_WARN("[CommandGate] switched to MANUAL_RETURN");
    return true;
  }

  bool setNormal(std_srvs::Trigger::Request&,
                 std_srvs::Trigger::Response& response) {
    gate_.switchToNormal();
    publishSource();
    response.success = true;
    response.message = "command source switched to NORMAL";
    ROS_WARN("[CommandGate] switched to NORMAL");
    return true;
  }

  void normalCallback(const mars_quadrotor_msgs::PositionCommandConstPtr& msg) {
    if (gate_.shouldForward(mrp::CommandSource::NORMAL)) {
      output_pub_.publish(*msg);
    }
  }

  void returnCallback(const mars_quadrotor_msgs::PositionCommandConstPtr& msg) {
    if (gate_.shouldForward(mrp::CommandSource::MANUAL_RETURN)) {
      output_pub_.publish(*msg);
    }
  }

  void publishSource() {
    std_msgs::String message;
    message.data = sourceName(gate_.source());
    source_pub_.publish(message);
  }

  static std::string sourceName(mrp::CommandSource source) {
    return source == mrp::CommandSource::MANUAL_RETURN ? "MANUAL_RETURN"
                                                       : "NORMAL";
  }

  ros::NodeHandle nh_, private_nh_;
  std::string normal_topic_, return_topic_, controller_topic_, source_topic_;
  ros::Subscriber normal_sub_, return_sub_;
  ros::Publisher output_pub_, source_pub_;
  ros::ServiceServer set_return_srv_, set_normal_srv_;
  mrp::CommandGateCore gate_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "manual_return_command_gate");
  CommandGateNode node;
  ros::spin();
  return 0;
}
