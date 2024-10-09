// Copyright (c) 2024, Ontoptech Inc.
// All rights reserved.
// Author: Rockey Shao
#ifndef BEEFAST_MESSAGE_PARSER__ODOM_HPP_
#define BEEFAST_MESSAGE_PARSER__ODOM_HPP_
#include <stdio.h>
#include <stdlib.h>
#include <atomic>
#include <string.h>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "beefast_serial/robot_serial.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <list>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "rclcpp/duration.hpp"
#include <functional>
#include <tf2/utils.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "sensor_msgs/msg/imu.hpp"
#include "beefast_interfaces/msg/motion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "beefast_message_parser/robot_message.hpp"
#include "tf2/convert.h"
#include "tf2/LinearMath/Transform.h"
#include <cmath>
#include <fstream>

namespace beefast_message_parser
{
    class OdomNode: public rclcpp::Node
    {
    public:
      OdomNode(std::string name);

      ~OdomNode();

      void motion_callback(const beefast_message_parser::MotionMessage& message, double secs);

      // void calculate_frequency();

      void load_params();

    private:
      std::string toString(double val, int precision);
      //参数定义
      std::shared_ptr<beefast_serial::RobotSerial> robot_serial_;

      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
      rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
      std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

      void ResetPostion();

      void pub_tf(double secs);

      void reset_motion_data();

      void set_motion_data(const beefast_message_parser::MotionMessage& message);

      void publish_odometry(double secs);

      void cumulate_odometry();

      void publish_imu(double secs);

      // imu add
      int load_acc_params(std::string path);
      int load_gyro_params(std::string path);
      int load_gravity_params(std::string path);
      int init();

      inline Eigen::Matrix<double, 3, 1> acc_correct(const Eigen::Matrix<double, 3, 1> &raw_data) const
      {
          return acc_ms_mat_*(raw_data - acc_bias_vec_);
      };

      inline Eigen::Matrix<double, 3, 1> gyro_correct(const Eigen::Matrix<double, 3, 1> &raw_data) const
      {
          return gyro_ms_mat_*(raw_data - gyro_bias_vec_);
      };

      // int16_t  left_encoder_;  // deltaT时间内左轮脉冲数增量(目前按20Hz 50ms计算deltaT)
      // int16_t  right_encoder_; // deltaT时间内右轮脉冲数增量(目前按20Hz 50ms计算deltaT)
      double velocity_left_;   //左速度  米/miao
      double velocity_right_;   //右速度
      // 以下为IMU传感器数据
      double  angle_x_;  // imu上报当前x轴角度
      double  angle_y_;  // imu上报当前y轴角度
      double  angle_z_;  // imu上报当前z轴角度

      double accel_x_ ;  // imu上报当前x轴线加速度
      double accel_y_ ;  // imu上报当前y轴线加速度
      double accel_z_ ;  // imu上报当前z轴线加速度

      double gyro_x_ ;  // imu上报当前x轴角速度
      double gyro_y_ ;  // imu上报当前y轴角速度
      double gyro_z_ ;  // imu上报当前z轴角速度

      double quat_x_;   // 四元组
      double quat_y_;
      double quat_z_;
      double quat_w_;

      // 初始化小车位置
      double x_ = 0.0;  // 当前x轴位移
      double y_ = 0.0;  // 当前y轴位移
      double th_ = 0.0; // 当前z轴Raw

      double vx_ = 0.0; // 当前x轴速度
      double vy_ = 0.0; // 当前y轴速度
      double vth_ = 0.0; // 当前z轴角速度

      double delta_distance_;
      double left_count_;
      double right_count_;

      std::string odom_topic_;
      std::string imu_topic_;
      std::string odom_angle_source_; // 里程计角度数据来源
      double wheel_diameter_; // 车轮直径
      // int pulse_resolution_; // 轮子转一圈编码器的分辨率
      double wheel_base_; //两轮之间的轴距
      int rate_;
      // imu inertial params


      // inertial params of acc
      Eigen::Matrix<double, 3, 3> acc_mis_mat_;
      Eigen::Matrix<double, 3, 3> acc_scale_mat_;
      Eigen::Matrix<double, 3, 1> acc_bias_vec_;
      Eigen::Matrix<double, 3, 3> acc_ms_mat_;

      // inertial params of gyro
      Eigen::Matrix<double, 3, 3> gyro_mis_mat_;
      Eigen::Matrix<double, 3, 3> gyro_scale_mat_;
      Eigen::Matrix<double, 3, 1> gyro_bias_vec_;
      Eigen::Matrix<double, 3, 3> gyro_ms_mat_;

      // gravity params
      Eigen::Vector3d actual_g_;
      int g_count_;

     int call_count_ = 0;

    std::thread timer_thread_;
      
    rclcpp::Time start_time_;

     rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr visual_publisher_;

     rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr odom_visual_publisher_;
    };
} // namespace beefast_message_parser

#endif //  
