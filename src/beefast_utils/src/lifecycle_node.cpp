// Copyright (c) 2024, Ontoptech Inc.
// All rights reserved.
// Author: Rockey Shao

#include "beefast_utils/lifecycle_node.hpp"

#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"

namespace beefast_utils
{

// The beefast_utils::LifecycleNode class is temporary until we get the
// required support for lifecycle nodes in MessageFilter, TransformListener,
// and TransforBroadcaster. 
// Until then, this class can provide a normal ROS node that has a thread
// that processes the node's messages. If a derived class needs to interface
// to one of these classes - MessageFilter, etc. - that don't yet support
// lifecycle nodes, it can simply set the use_rclcpp_node flag in the
// constructor and then provide the rclcpp_node_ to the helper classes, like
// MessageFilter.
//

LifecycleNode::LifecycleNode(
  const std::string & node_name,
  const std::string & ns, bool use_rclcpp_node,
  const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(node_name, ns, options),
  use_rclcpp_node_(use_rclcpp_node)
{
  // server side never times out from lifecycle manager
  this->declare_parameter(bond::msg::Constants::DISABLE_HEARTBEAT_TIMEOUT_PARAM, true);
  this->set_parameter(
    rclcpp::Parameter(
      bond::msg::Constants::DISABLE_HEARTBEAT_TIMEOUT_PARAM, true));

  if (use_rclcpp_node_) {
    std::vector<std::string> new_args = options.arguments();
    new_args.push_back("--ros-args");
    new_args.push_back("-r");
    new_args.push_back(std::string("__node:=") + this->get_name() + "_rclcpp_node");
    new_args.push_back("--");
    rclcpp_node_ = std::make_shared<rclcpp::Node>(
      "_", ns, rclcpp::NodeOptions(options).arguments(new_args));
    rclcpp_thread_ = std::make_unique<NodeThread>(rclcpp_node_);
  }

  print_lifecycle_node_notification();
}

LifecycleNode::~LifecycleNode()
{
  RCLCPP_INFO(get_logger(), "Destroying");
  // In case this lifecycle node wasn't properly shut down, do it here
  if (get_current_state().id() ==
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    on_deactivate(get_current_state());
    on_cleanup(get_current_state());
  }
}

void LifecycleNode::createBond()
{
  RCLCPP_INFO(get_logger(), "Creating bond (%s) to lifecycle manager.", this->get_name());

  bond_ = std::make_unique<bond::Bond>(
    std::string("bond"),
    this->get_name(),
    shared_from_this());

  bond_->setHeartbeatPeriod(0.10);
  bond_->setHeartbeatTimeout(4.0);
  bond_->start();
}

void LifecycleNode::destroyBond()
{
  RCLCPP_INFO(get_logger(), "Destroying bond (%s) to lifecycle manager.", this->get_name());

  if (bond_) {
    bond_.reset();
  }
}

void LifecycleNode::print_lifecycle_node_notification()
{
  RCLCPP_INFO(
    get_logger(),
    "\n\t%s lifecycle node launched. \n"
    "\tWaiting on external lifecycle transitions to activate\n"
    "\tSee https://design.ros2.org/articles/node_lifecycle.html for more information.", get_name());
}

}  // namespace beefast_utils
