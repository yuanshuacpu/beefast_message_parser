// Copyright (c) 2024, Ontoptech Inc.
// All rights reserved.
// Author: Rockey Shao

#ifndef BEEFAST_UTILS__LIFECYCLE_SERVICE_CLIENT_HPP_
#define BEEFAST_UTILS__LIFECYCLE_SERVICE_CLIENT_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "beefast_utils/node_utils.hpp"
#include "beefast_utils/service_client.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

namespace beefast_utils {

/// Helper functions to interact with a lifecycle node.
class LifecycleServiceClient {
public:
  explicit LifecycleServiceClient(const std::string &lifecycle_node_name);

  LifecycleServiceClient(const std::string &lifecycle_node_name,
                         rclcpp::Node::SharedPtr parent_node);

  /// Trigger a state change
  /**
   * Throws std::runtime_error on failure
   */
  void change_state(
      const uint8_t transition, // takes a lifecycle_msgs::msg::Transition id
      const std::chrono::seconds timeout);

  /// Trigger a state change, returning result
  bool change_state(std::uint8_t transition);

  /// Get the current state as a lifecycle_msgs::msg::State id value
  /**
   * Throws std::runtime_error on failure
   */
  uint8_t
  get_state(const std::chrono::seconds timeout = std::chrono::seconds::max());

protected:
  rclcpp::Node::SharedPtr node_;
  ServiceClient<lifecycle_msgs::srv::ChangeState> change_state_;
  ServiceClient<lifecycle_msgs::srv::GetState> get_state_;
};

} // namespace beefast_utils

#endif // BEEFAST_UTILS__LIFECYCLE_SERVICE_CLIENT_HPP_
