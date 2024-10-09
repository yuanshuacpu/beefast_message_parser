// Copyright (c) 2024, Ontoptech Inc.
// All rights reserved.
// Author: Rockey Shao

#ifndef BEEFAST_MESSAGE_PARSER__MESSAGE_PROCESSOR_HPP_
#define BEEFAST_MESSAGE_PARSER__MESSAGE_PROCESSOR_HPP_

#include "beefast_interfaces/msg/motion.hpp"
#include "beefast_interfaces/msg/motion_with_collision.hpp"
#include "beefast_serial/robot_serial.hpp"
#include "beefast_message_parser/robot_message.hpp"
#include "beefast_message_parser/odom.hpp"
#include "beefast_utils/string_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <chrono>
#include <sstream>
#include <string>
#include <vector>
#include <memory>  
#include <queue>
#include <deque>  
#include <mutex>  
#include <condition_variable>  
#include <thread>

namespace beefast_message_parser {

// struct __attribute__((packed)) SerialMessage {
struct SerialMessage {
    uint16_t length; 
    uint8_t func_id;  
    double recv_secs;
    std::vector<uint8_t> data;
    int num;
    SerialMessage(uint8_t len, uint8_t funid, double secs,int nums) : length(len), func_id(funid),recv_secs(secs), data(len+1),num(nums) {
      //RCLCPP_INFO(rclcpp::get_logger("message_processor"), "1serial message len %d, data size:%d, capacity:%ld ,length, data.size(), data.capacity());
    } 
};

// 使用unique_ptr来管理Message对象的内存  
using MessagePtr = std::unique_ptr<beefast_message_parser::SerialMessage>; 

enum class MatchKind : uint8_t {
  kMatchFF,
  kMatchFC,
  kMatchMeta,
  kMatchBody
};

class MessageProcessor : public rclcpp::Node {
public:
  MessageProcessor(std::string name, std::string port = "/dev/ttyS3",
                   uint32_t baudrate = 921600, int recv_msg_interval = 100);

  ~MessageProcessor();
  
  void parse_message();

protected:
  void process_motion_msg(const std::vector<uint8_t> &message, double secs);

  void process_collision_msg(const std::vector<uint8_t> &message);

  std::vector<uint8_t> hex_string_to_bytes(const std::string &hex);

  std::string bytes_to_hex_string(const std::vector<uint8_t> &data);


  void read_message_from_board();

  void process_message(beefast_message_parser::MessagePtr&& message) ;

private:
  rclcpp::TimerBase::SharedPtr recv_board_msg_timer_;
  std::shared_ptr<beefast_serial::RobotSerial> robot_serial_;
  void recv_message_from_board();
  rclcpp::Publisher<beefast_interfaces::msg::Motion>::SharedPtr
      motion_publisher_;

  rclcpp::Publisher<beefast_interfaces::msg::MotionWithCollision>::SharedPtr collision_publisher_;

  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr visual_publisher_;

  std::shared_ptr<serial::Serial> serial_;
  // std::queue<beefast_message_parser::SerialMessage> message_queue_;
  // std::queue<MessagePtr> message_queue_;
  std::deque<MessagePtr> message_queue_; 
  std::mutex queue_mutex_; 
  std::thread recv_thread_;
  std::vector<std::thread> pool_;
  const size_t num_threads_ = 2;
  std::condition_variable cond_;

  std::shared_ptr<beefast_message_parser::OdomNode> odom_node_;
};

} // namespace beefast_message_parser

#endif
