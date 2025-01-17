// Copyright (c) 2024, Ontoptech Inc.
// All rights reserved.
// Author: Rockey Shao

#ifndef BEEFAST_UTILS__LIFECYCLE_NODE_HPP_
#define BEEFAST_UTILS__LIFECYCLE_NODE_HPP_

#include <memory>
#include <string>
#include <thread>

#include "beefast_utils/node_thread.hpp"
#include "bond/msg/constants.hpp"
#include "bondcpp/bond.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace beefast_utils {

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// The following is a temporary wrapper for rclcpp_lifecycle::LifecycleNode.
// This class adds the optional creation of an rclcpp::Node that can be used by
// derived classes to interface to classes, such as MessageFilter and
// TransformListener, that don't yet support lifecycle nodes. Once we get the
// fixes into ROS2, this class will be removed.

/**
 * @class beefast_utils::LifecycleNode
 * @brief A lifecycle node wrapper to enable common Nav2 needs such as
 * background node threads and manipulating parameters
 */
class LifecycleNode : public rclcpp_lifecycle::LifecycleNode {
public:
  /**
   * @brief A lifecycle node constructor
   * @param node_name Name for the node
   * @param namespace Namespace for the node, if any
   * @param use_rclcpp_node Whether to create an internal client node
   * @param options Node options
   */
  LifecycleNode(const std::string &node_name, const std::string &ns = "",
                bool use_rclcpp_node = false,
                const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  virtual ~LifecycleNode();

  typedef struct {
    double from_value;
    double to_value;
    double step;
  } floating_point_range;

  typedef struct {
    int from_value;
    int to_value;
    int step;
  } integer_range;

  /**
   * @brief Declare a parameter that has no integer or floating point range
   * constraints
   * @param node_name Name of parameter
   * @param default_value Default node value to add
   * @param description Node description
   * @param additional_constraints Any additional constraints on the parameters
   * to list
   * @param read_only Whether this param should be considered read only
   */
  void add_parameter(const std::string &name,
                     const rclcpp::ParameterValue &default_value,
                     const std::string &description = "",
                     const std::string &additional_constraints = "",
                     bool read_only = false) {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.name = name;
    descriptor.description = description;
    descriptor.additional_constraints = additional_constraints;
    descriptor.read_only = read_only;

    declare_parameter(descriptor.name, default_value, descriptor);
  }

  /**
   * @brief Declare a parameter that has a floating point range constraint
   * @param node_name Name of parameter
   * @param default_value Default node value to add
   * @param fp_range floating point range
   * @param description Node description
   * @param additional_constraints Any additional constraints on the parameters
   * to list
   * @param read_only Whether this param should be considered read only
   */
  void add_parameter(const std::string &name,
                     const rclcpp::ParameterValue &default_value,
                     const floating_point_range fp_range,
                     const std::string &description = "",
                     const std::string &additional_constraints = "",
                     bool read_only = false) {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.name = name;
    descriptor.description = description;
    descriptor.additional_constraints = additional_constraints;
    descriptor.read_only = read_only;
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = fp_range.from_value;
    descriptor.floating_point_range[0].to_value = fp_range.to_value;
    descriptor.floating_point_range[0].step = fp_range.step;

    declare_parameter(descriptor.name, default_value, descriptor);
  }

  /**
   * @brief Declare a parameter that has an integer range constraint
   * @param node_name Name of parameter
   * @param default_value Default node value to add
   * @param integer_range Integer range
   * @param description Node description
   * @param additional_constraints Any additional constraints on the parameters
   * to list
   * @param read_only Whether this param should be considered read only
   */
  void add_parameter(const std::string &name,
                     const rclcpp::ParameterValue &default_value,
                     const integer_range int_range,
                     const std::string &description = "",
                     const std::string &additional_constraints = "",
                     bool read_only = false) {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.name = name;
    descriptor.description = description;
    descriptor.additional_constraints = additional_constraints;
    descriptor.read_only = read_only;
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = int_range.from_value;
    descriptor.integer_range[0].to_value = int_range.to_value;
    descriptor.integer_range[0].step = int_range.step;

    declare_parameter(descriptor.name, default_value, descriptor);
  }

  /**
   * @brief Get a shared pointer of this
   */
  std::shared_ptr<beefast_utils::LifecycleNode> shared_from_this() {
    return std::static_pointer_cast<beefast_utils::LifecycleNode>(
        rclcpp_lifecycle::LifecycleNode::shared_from_this());
  }

  /**
   * @brief Abstracted on_error state transition callback, since unimplemented
   * as of 2020 in the managed ROS2 node state machine
   * @param state State prior to error transition
   * @return Return type for success or failed transition to error state
   */
  beefast_utils::CallbackReturn
  on_error(const rclcpp_lifecycle::State & /*state*/) {
    RCLCPP_FATAL(get_logger(),
                 "Lifecycle node %s does not have error state implemented",
                 get_name());
    return beefast_utils::CallbackReturn::SUCCESS;
  }

  /**
   * @brief Create bond connection to lifecycle manager
   */
  void createBond();

  /**
   * @brief Destroy bond connection to lifecycle manager
   */
  void destroyBond();

protected:
  /**
   * @brief Print notifications for lifecycle node
   */
  void print_lifecycle_node_notification();

  // Whether or not to create a local rclcpp::Node which can be used for ROS2
  // classes that don't yet support lifecycle nodes
  bool use_rclcpp_node_;

  // The local node
  rclcpp::Node::SharedPtr rclcpp_node_;

  // When creating a local node, this class will launch a separate thread
  // created to spin the node
  std::unique_ptr<NodeThread> rclcpp_thread_;

  // Connection to tell that server is still up
  std::unique_ptr<bond::Bond> bond_{nullptr};
};

} // namespace beefast_utils

#endif // BEEFAST_UTILS__LIFECYCLE_NODE_HPP_
