// Copyright (c) 2024, Ontoptech Inc.
// All rights reserved.
// Author: Rockey Shao

#include "beefast_utils/lifecycle_service_client.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

using beefast_utils::generate_internal_node;
using std::make_shared;
using std::string;
using std::chrono::seconds;

namespace beefast_utils {

LifecycleServiceClient::LifecycleServiceClient(
    const string &lifecycle_node_name)
    : node_(generate_internal_node(lifecycle_node_name + "_lifecycle_client")),
      change_state_(lifecycle_node_name + "/change_state", node_),
      get_state_(lifecycle_node_name + "/get_state", node_) {}

LifecycleServiceClient::LifecycleServiceClient(
    const string &lifecycle_node_name, rclcpp::Node::SharedPtr parent_node)
    : node_(parent_node),
      change_state_(lifecycle_node_name + "/change_state", node_),
      get_state_(lifecycle_node_name + "/get_state", node_) {}

void LifecycleServiceClient::change_state(const uint8_t transition,
                                          const seconds timeout) {
  change_state_.wait_for_service(timeout);
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  change_state_.invoke(request, timeout);
}

bool LifecycleServiceClient::change_state(std::uint8_t transition) {
  change_state_.wait_for_service();
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  auto response =
      std::make_shared<lifecycle_msgs::srv::ChangeState::Response>();
  request->transition.id = transition;
  return change_state_.invoke(request, response);
}

uint8_t LifecycleServiceClient::get_state(const seconds timeout) {
  get_state_.wait_for_service(timeout);
  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto result = get_state_.invoke(request, timeout);
  return result->current_state.id;
}

} // namespace beefast_utils
