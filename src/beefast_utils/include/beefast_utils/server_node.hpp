// Copyright (c) 2024, Ontoptech Inc.
// All rights reserved.
// Author: Rockey Shao

#ifndef BEEFAST_UTILS__SERVER_NODE_HPP_
#define BEEFAST_UTILS__SERVER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <string>

namespace beefast_utils {
class ServerNode : public rclcpp::Node {
public:
  ServerNode(const std::string &node_name, const std::string &ns = "",
             const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : rclcpp::Node(node_name, ns, options) {}

  virtual ~ServerNode() {}

  // @brief 开始节点  先配置节点，再激活节点
  // @return bool 开始节点返回true，否则返回false
  virtual bool Startup() = 0;

  // @brief 配置，激活，去激活，清理，关闭
  // @return bool 开始节点返回true，否则返回false
  virtual bool Configure() = 0;

  // @brief 激活
  // @return bool 开始节点返回true，否则返回false
  virtual bool Activate() = 0;

  // @brief 去激活
  // @return bool 开始节点返回true，否则返回false
  virtual bool Deactivate() = 0;

  // @brief 清理
  // @return bool 开始节点返回true，否则返回false
  virtual bool Cleanup() = 0;

  // @brief 关闭
  // @return bool 开始节点返回true，否则返回false
  virtual bool Shutdown() = 0;

  // @brief 获取service节点当前状态
  // @return uint8_t
  virtual uint8_t GetNodeState() = 0;
};

} // namespace beefast_utils
#endif // BEEFAST_UTILS__SERVER_NODE_HPP_