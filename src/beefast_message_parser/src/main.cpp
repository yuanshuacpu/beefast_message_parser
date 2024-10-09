#include "beefast_message_parser/message_processor.hpp"

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);

//   // #ifdef DEBUG_MODE_ON  
//   //   std::cout << "Debug mode is on." << std::endl;
//   // #else  
//     // std::cout << "Debug mode is off." << std::endl; 
//     auto node_message_recv = std::make_shared<beefast_message_parser::MessageProcessor>("message_precessor");
//     rclcpp::spin(node_message_recv);
//   // #endif
//   rclcpp::shutdown();
//   return 0;
// }

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // #ifdef DEBUG_MODE_ON  
  //   std::cout << "Debug mode is on." << std::endl;
  // #else  
    // std::cout << "Debug mode is off." << std::endl; 
  auto node_message_recv = std::make_shared<beefast_message_parser::MessageProcessor>("message_precessor");
  node_message_recv->parse_message();
  rclcpp::spin(node_message_recv);
  // #endif
  rclcpp::shutdown();
  return 0;
}