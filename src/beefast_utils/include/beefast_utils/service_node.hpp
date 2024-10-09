// Copyright (c) 2024, Ontoptech Inc.
// All rights reserved.
// Author: Rockey Shao

#ifndef BEEFAST_UTILS__SERVICE_NODE_HPP_
#define BEEFAST_UTILS__SERVICE_NODE_HPP_

#include <iostream>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"

namespace beefast_utils {
class ServiceNode : public rclcpp::Node {
private:
  static std::shared_ptr<ServiceNode> instance;

  ServiceNode(std::string name);

protected:
  static std::shared_ptr<ServiceNode> getInstance();

public:
  virtual ~ServiceNode();

  void useResource();
};

} // namespace beefast_utils
#endif // BEEFAST_UTILS__SERVICE_NODE_HPP_