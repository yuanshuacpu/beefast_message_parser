#include "beefast_message_parser/odom.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto  odom_node = std::make_shared<beefast_message_parser::OdomNode>("odom_imu_pub");
  std::string param_path = "/opt/beefast/data/params/beefast.yaml";
  auto param_client =
    std::make_shared<rclcpp::SyncParametersClient>(odom_node);
  auto load_future = param_client->load_parameters(param_path);

  for (auto &item : load_future) {
    RCLCPP_INFO(rclcpp::get_logger("demo"), "set status:%d, reason: %s", item.successful,
                item.reason.c_str());
  }
  auto list_parameters = param_client->list_parameters({}, 1);
  RCLCPP_INFO(rclcpp::get_logger("demo"),"list params size :%ld",list_parameters.names.size());
  odom_node->load_params();
  rclcpp::spin(odom_node);
  rclcpp::shutdown();
  return 0;
}