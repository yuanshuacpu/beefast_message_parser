// Copyright (c) 2024, Ontoptech Inc.
// All rights reserved.
// Author: Rockey Shao

#include "beefast_utils/service_node.hpp"

namespace beefast_utils {
std::shared_ptr<ServiceNode> ServiceNode::instance = nullptr;

ServiceNode::ServiceNode(std::string name) {
  RCLCPP_INFO(get_logger(), "ServiceNode initialized");
}

ServiceNode::~ServiceNode() override{
  RCLCPP_INFO(get_logger(), "ServiceNode destroyed");
  rclcpp::Node:~Node();
}

std::shared_ptr<ServiceNode> ServiceNode::getInstance() {
  if (!instance) {
    RCLCPP_INFO(get_logger(), "ServiceNode create new instance");
    instance = std::shared_ptr<ServiceNode>(new ServiceNode());
  }
  return instance;
}

void ServiceNode::useResource() {
  std::cout << "Resource being used" << std::endl;
  RCLCPP_INFO(get_logger(), "Resource being used");
}

} // beefast_utils

int main() {
  std::shared_ptr<ServiceNode> resourceManager = ServiceNode::getInstance();
  resourceManager->useResource();

  // 手动释放资源
  resourceManager.reset();

  RCLCPP_INFO(get_logger(), "Resource manually released");

  return 0;
}