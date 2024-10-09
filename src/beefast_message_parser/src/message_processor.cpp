#include "beefast_message_parser/message_processor.hpp"
#include<fstream>
#include<iostream>
#include <stdexcept>

namespace beefast_message_parser {
  //可注释  测试所用
  double secs2;
  double secs3;
uint32_t receive_sum = {};
uint32_t discard_sum = {};
int count = {};
// int32_t left_sum_encoder_1 = {};
// int32_t right_sum_encoder_1= {};
// int32_t left_sum_encoder_2 = {};
// int32_t right_sum_encoder_2 = {};
int32_t count_publish = {};

MessageProcessor::MessageProcessor(std::string name, std::string port,
                                   uint32_t baudrate, int recv_msg_interval)
    : rclcpp::Node(name) {
  // robot_serial_ = std::make_shared<beefast_serial::RobotSerial>("/dev/ttyS3",
  // 115200);
  auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable();
  RCLCPP_INFO(get_logger(),"MessageProcessor constructor");

  // motion_publisher_ =
  //  create_publisher<beefast_interfaces::msg::Motion>("motion_from_board",
  //  custom_qos);

  collision_publisher_ =
    create_publisher<beefast_interfaces::msg::MotionWithCollision>("collsion_alarm", 
    custom_qos);

  visual_publisher_ = create_publisher<geometry_msgs::msg::TransformStamped>("/message_debug", custom_qos);
  #ifdef DEBUG_MODE_ON  
    std::cout << "Debug mode MessageProcessor serial not used." << std::endl;
  #else 
    RCLCPP_INFO(get_logger(),"list param 1110");
    serial_ = std::make_shared<serial::Serial>(
        port, baudrate, serial::Timeout::simpleTimeout(5000));
    serial_->flushInput();
  #endif

  RCLCPP_INFO(get_logger(),"list param 1111");

  try {
    odom_node_ = std::make_shared<beefast_message_parser::OdomNode>("odom_imu_pub");   //1

    std::string param_path = "/opt/beefast/data/params/beefast.yaml";
    auto param_client =
      std::make_shared<rclcpp::SyncParametersClient>(odom_node_);
    while(!param_client->wait_for_service(std::chrono::seconds(1))) 
    {
      RCLCPP_INFO(get_logger(),"waiting for service");
    }
    auto load_future = param_client->load_parameters(param_path);
    odom_node_->load_params();
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(this->get_logger(), "Caught std::exception: %s", ex.what());
  }

  // for (auto &item : load_future) {
  //   RCLCPP_INFO(get_logger(), "set status:%d, reason: %s", item.successful,
  //               item.reason.c_str());
  // }

  RCLCPP_INFO(get_logger(),"list param 1112");
  // 分离线程，使其在后台运行
  recv_thread_ = std::thread(&MessageProcessor::read_message_from_board, this);   //2
  // recv_thread_.detach();
  // 初始化线程池

  // for (size_t i = 0; i < num_threads_; ++i) {
  //   pool_.emplace_back([this]() {
  //     // RCLCPP_INFO(get_logger(),"list param 1114");
  //     this->parse_message();     //3
  //   });
  // }

  // RCLCPP_INFO(get_logger(),"list param 1115");
  // auto list_parameters = param_client->list_parameters({}, 1);
  // RCLCPP_INFO(get_logger(),"list params size :%ld",list_parameters.names.size());
  // for (auto param:list_parameters.names) {
  //   RCLCPP_INFO(get_logger(),"params %s",param.c_str());
  // }
}

MessageProcessor::~MessageProcessor() { recv_thread_.join(); }

void MessageProcessor::read_message_from_board() {       //2
  // RCLCPP_INFO(get_logger(),"list param 11111");
  if (serial_->isOpen()) {
    uint8_t header_ff[1];
    uint8_t header_fc[1];
    uint8_t metadata[3];
    uint8_t function;
    uint16_t leng;
    MatchKind next_match = MatchKind::kMatchFF;
    if (serial_->waitReadable()) 
    {
      while (rclcpp::ok()) {
        std::string start_flag;
        switch (next_match) {
          case MatchKind::kMatchFF:
            header_ff[0] = 0;
            serial_->read(header_ff, 1);
            if (header_ff[0] == 0xff) {
              // RCLCPP_INFO(rclcpp::get_logger("message_processor"),
              //             "FF message header detected");
              next_match = MatchKind::kMatchFC;
            }
            break;
          case MatchKind::kMatchFC:
            header_fc[0] = 0;
            serial_->read(header_fc, 1);
            if (header_fc[0] == 0xfc) {
              // RCLCPP_INFO(rclcpp::get_logger("message_processor"),
              //             "FC message header detected");
              next_match = MatchKind::kMatchMeta;
            } else {
              next_match = MatchKind::kMatchFF;
            }
            break;
          case MatchKind::kMatchMeta:
            metadata[0] = 0;   
            metadata[1] = 0;
            metadata[2] = 0;
            serial_->read(metadata, 3);
            function = metadata[0];
            leng = (metadata[2] << 8) | metadata[1];
            if (function ==beefast_message_parser::kMessageMotionFunctionId ||    
                function ==beefast_message_parser::kMessageAlarmFunctionId) {
              next_match = MatchKind::kMatchBody;
            } else if (metadata[0] == 0xff && metadata[1] == 0xfc) {
              next_match = MatchKind::kMatchMeta;
	      std::fstream fout;
	      fout.open("/home/sunrise/beefast/test_data.txt",std::ios::out|std::ios::app);
              fout<< beefast_utils::format_time(this->get_clock()->now())<<"ff&&fc invalid funcid"<<std::endl;
	      fout.close();
            } else if (metadata[1] == 0xff) {
              next_match = MatchKind::kMatchFC;
	      std::fstream fout;
              fout.open("/home/sunrise/beefast/test_data.txt",std::ios::out|std::ios::app);
              fout<< beefast_utils::format_time(this->get_clock()->now())<<"ff check fc"<<std::endl;
              fout.close();
            } else {
              next_match = MatchKind::kMatchFF;
	      std::fstream fout;
              fout.open("/home/sunrise/beefast/test_data.txt",std::ios::out|std::ios::app);
              fout<< beefast_utils::format_time(this->get_clock()->now())<<"going to check ff"<<std::endl;
              fout.close();
            }
            // next_match = MatchKind::kMatchBody;
            // RCLCPP_INFO(rclcpp::get_logger("message_processor"),"Meta message length:%d func:%d",metadata[0], metadata[1]);
            break;
          case MatchKind::kMatchBody:
            if (leng > 100) {
              std::fstream fout;
              
              fout.open("/home/sunrise/beefast/test_data.txt",std::ios::out|std::ios::app);
              fout<< beefast_utils::format_time(this->get_clock()->now())<<"length :"<<std::to_string(metadata[0])<<"func :"<<std::to_string(metadata[1])<<std::endl;
              RCLCPP_ERROR(rclcpp::get_logger("message_processor"), "invalid serial message length:%d, funcId:%d",leng, function);
              // std::vector<uint8_t> data;
              uint8_t data1[10];
              size_t size = serial_->read(data1,10);
              // std::string hex_string = bytes_to_hex_string(data1);
              // fout<<"func:"<<std::to_string(metadata[0])<<"len:"<<std::to_string(metadata[1])<<"last 100 message:"<<hex_string.c_str()<<std::endl;
              fout.close();
                // RCLCPP_ERROR(rclcpp::get_logger("message_processor"),
                //             "func:%x len:%d, last 100 message: %s",
                //             function, leng,
                //             hex_string.c_str());
            } else {
              double secs = this->get_clock()->now().seconds();
              count++;
              RCLCPP_INFO(rclcpp::get_logger("message1"),"time : %f count : %d",secs,count);
              beefast_message_parser::MessagePtr read_message =
                std::make_unique<beefast_message_parser::SerialMessage>(
                    leng, function, secs,count);
              // RCLCPP_INFO(rclcpp::get_logger("message_processor"), "2len:%d data size:%d, capacity:%ld",read_message->length, leng, read_message->data.capacity());
              size_t size = serial_->read(read_message->data.data(),       //读进去
                                          read_message->data.capacity());
              // RCLCPP_INFO(rclcpp::get_logger("message_processor"), "len:%d data size:%d, capacity:%ld",read_message->length, read_message->data.size(), read_message->data.capacity());
              if (metadata[1] == kMessageAlarmFunctionId) {     //怎么读进去
                std::unique_lock<std::mutex> lock(queue_mutex_);
                // 告警信息优先级最高
                message_queue_.push_front(std::move(read_message));
                lock.unlock();
                this->cond_.notify_one();
              } else {
                  std::unique_lock<std::mutex> lock(queue_mutex_);
                  message_queue_.push_back(std::move(read_message));
                  lock.unlock();
                  this->cond_.notify_one();
              }

            }

            // RCLCPP_INFO(rclcpp::get_logger("message_processor"),
            //   "message count %ld", message_queue_.size());
            next_match = MatchKind::kMatchFF;
            break;
        }
      }
      // RCLCPP_INFO(rclcpp::get_logger("message_processor"), "serial data  over");
    }
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void MessageProcessor::parse_message() {
  // 取出并处理队列中的消息，直到队列为空或达到某个限制
  while (rclcpp::ok()) 
  {
    double secs21 = this->get_clock()->now().seconds();
    MessagePtr read_message;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      while (message_queue_.empty())
      this->cond_.wait(lock);
      read_message = std::move(message_queue_.front());
      message_queue_.pop_front();
      lock.unlock();
    }
    // double secs23 = this->get_clock()->now().seconds();
    // RCLCPP_INFO(rclcpp::get_logger("MessageProcessor"),"parse_message wait : %f",secs23 -secs21);
    // RCLCPP_INFO(get_logger(),"list param 1115");
    process_message(std::move(read_message));
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // double secs22 = this->get_clock()->now().seconds();
    // RCLCPP_INFO(rclcpp::get_logger("MessageProcessor"),"parse_message : %f",secs22 -secs21);
  }
}

void MessageProcessor::process_message(
  beefast_message_parser::MessagePtr&& message) 
{
  // RCLCPP_INFO(get_logger(),"list param 1116");
  // secs2 = this->get_clock()->now().seconds();
  // RCLCPP_INFO(rclcpp::get_logger("2messsge"),"time secs2 : %f , count : %d",secs2 , message->num);
  const std::vector<uint8_t>& message_data = message->data;
  uint8_t sum_data = {};
  if(message->func_id == kMessageMotionFunctionId)
  {
    sum_data = 0x02+0x26;
  }
  else if (message->func_id == kMessageAlarmFunctionId)
  {
    sum_data = 0x60+0x01;
  }
  
  
  //测试 打印所有data域元素。
  for(int i = 0;i<message_data.size() -1;i++)
  {
    // RCLCPP_INFO(rclcpp::get_logger("COTENT"),"message_data[%d]:%d",i,message_data[i]);
    sum_data += message_data[i];
  }
  //测试速度
  int16_t  velocity_left =  static_cast<double>( static_cast<uint16_t>(message_data[1]) <<8 | (static_cast<uint16_t>(message_data[0])) );
  int16_t  velocity_right = static_cast<double>( static_cast<uint16_t>(message_data[3]) <<8 | (static_cast<uint16_t>(message_data[2])) );

  // left_sum_encoder_1 += left_lun;       //需要修改的解析数据  编码器
  // right_sum_encoder_1 += right_lun;

  // 校验和正确
  if( sum_data == message_data[message_data.size()-1])
  {
      receive_sum++;
      //测试 速度
      //RCLCPP_INFO(rclcpp::get_logger("接受消息数："),"receive_sum %d  左轮：%d 右轮：%d sum: %d", 
       //receive_sum,velocity_left,velocity_right , SerialMessage->sum);
       // if (receive_sum % 50 == 0) {
      //   RCLCPP_INFO(rclcpp::get_logger("接受消息数："),"receive_sum %d 当前序列号: %d 左轮：%d 右轮：%d ",
      //     receive_sum,message_data[message_data.size()-2],velocity_left,velocity_right);
      // }
      
      // RCLCPP_INFO(get_logger(),"list param 1117");
      switch (message->func_id) {
          case  beefast_message_parser::kMessageMotionFunctionId:
            // secs3 = this->get_clock()->now().seconds();
            // RCLCPP_INFO(rclcpp::get_logger("3messsge"),"time secs3 : %f",secs3);
            process_motion_msg(message->data, message->recv_secs);
            break;
          case  beefast_message_parser::kMessageAlarmFunctionId:
            process_collision_msg(message->data);
            break;
        }
    }
    else
    {
      discard_sum++;
      RCLCPP_INFO(rclcpp::get_logger("丢弃消息数"),"discard_sum %d 当前序列号： %d 左轮:%d 右轮:%d",discard_sum,message_data[message_data.size()-2],velocity_left,velocity_right);
      RCLCPP_INFO(rclcpp::get_logger("error_messager"),"check:%x sum_data:%x",message_data[message_data.size()-1],sum_data);
    }
}

std::vector<uint8_t> MessageProcessor::hex_string_to_bytes(
    const std::string& hex) {
  std::vector<uint8_t> bytes;
  std::istringstream isstream(hex);
  isstream >> std::hex;

  for (size_t i = 0; i < hex.size(); i += 2) {
    std::string byte_str = hex.substr(i, 2);
    uint8_t byte = std::stoi(byte_str, nullptr, 16);
    bytes.push_back(byte);
  }
  for (const auto& byte : bytes) {
    std::cout << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<int>(byte) << ' ';
  }
  return bytes;
}

std::string MessageProcessor::bytes_to_hex_string(
    const std::vector<uint8_t>& data) {
  std::stringstream ss;
  ss << std::hex << std::setfill('0');
  for (const auto& byte : data) {
    ss << std::setw(2) << static_cast<int>(byte);
  }
  return ss.str();
}

void MessageProcessor::process_motion_msg(const std::vector<uint8_t>& message, double secs) {

  
  beefast_message_parser::MotionMessage motion_msg;
  memset(&motion_msg, 0, sizeof(motion_msg));
  memcpy(&motion_msg, message.data(), sizeof(motion_msg));

  if (motion_msg.velocity_left > -1000 && motion_msg.velocity_left < 1000 &&
    motion_msg.velocity_right > -1000 && motion_msg.velocity_right < 1000) {

    count_publish++;
    // double secs4 = this->get_clock()->now().seconds();
    // RCLCPP_INFO(rclcpp::get_logger("4messsge"),"time secs4 : %f",secs4);
    odom_node_->motion_callback(motion_msg, secs);
    //  double secs41 = this->get_clock()->now().seconds();
    //  RCLCPP_INFO(rclcpp::get_logger("MessageProcessor"),"process_motion_msg : %f",secs41 -secs4);
    
    // if (receive_sum % 50 == 0) {
    //   RCLCPP_INFO(rclcpp::get_logger("接受消息数："),"receive_sum %d  左轮：%f 右轮：%f ",
    //     receive_sum,motion_msg.velocity_left / 1000.0,motion_msg.velocity_right/1000.0) ;
    // }
    // geometry_msgs::msg::TransformStamped visual_msg;
    // visual_msg.header.stamp = this->get_clock()->now();
    // visual_msg.transform.translation.x = receive_sum;
    // visual_msg.transform.translation.y = motion_msg.velocity_left;
    // visual_msg.transform.translation.z = motion_msg.velocity_right;
    // visual_msg.transform.rotation.x = count_publish;
    // visual_msg.transform.rotation.y = motion_msg.angle_x; 
    // visual_msg.transform.rotation.z = motion_msg.angle_z;
    // visual_msg.transform.rotation.w = discard_sum;
    // visual_publisher_->publish(visual_msg);

  } 
  // else {
  //   RCLCPP_ERROR(rclcpp::get_logger("message_processor"), "left encoder is invalid :%d, right encoder:%d",motion_msg.left_encoder, motion_msg.right_encoder);
  //   std::fstream f ;
  //   f.open("/home/sunrise/beefast/test_data.txt",std::ios::out|std::ios::app);
  //   auto t = std::chrono::system_clock::now(); 
  //   time_t tt = std::chrono::system_clock::to_time_t ( t );
  //   f<< "time :"<< tt <<"left encoder :"<<std::to_string(motion_msg.left_encoder)<<"right encoder :"<<std::to_string(motion_msg.right_encoder)<<std::endl;
  //   f.close();
  // }

 }
 

void MessageProcessor::process_collision_msg(const std::vector<uint8_t> &message) {

      RCLCPP_INFO(rclcpp::get_logger("message_processor"),
                    "start to process alarm message");
        beefast_message_parser::AlarmMessage alarm_msg;
        memset(&alarm_msg, 0, sizeof(alarm_msg));
        memcpy(&alarm_msg, message.data(), sizeof(alarm_msg));
        uint8_t collision_state = alarm_msg.collsion_data;

        // 检查左传感器是否有数据（检查第1位，即最低位）,1 对应于左传感器的位（最低位） 
        bool left_sensor = collision_state & 1;  
        // 检查右传感器是否有数据（检查第2位）,右传感器是左传感器位的下一位，所以右移一位再检查 
        bool right_sensor = (collision_state >> 1) & 1; 

        auto msg = beefast_interfaces::msg::MotionWithCollision();
        if (left_sensor && right_sensor) {
            // 两个传感器都有数据，向后退
            msg.direction = beefast_message_parser::kBackwardAndRight;
            msg.distance  = 0.18 * 2;
            msg.angle = -60;
            RCLCPP_INFO(rclcpp::get_logger("message_processor"),"两个传感器都有数据，向后退！");
        } else if (left_sensor) {  
            // 只有左传感器有数据，向右转  
            msg.direction = beefast_message_parser::kBackwardAndRight; 
            msg.distance  = 0.12;
            msg.angle = -20; 
            RCLCPP_INFO(rclcpp::get_logger("message_processor"),"只有左传感器有数据，向右转！");
        } else if (right_sensor) {  
            // 只有右传感器有数据，向左转  
            msg.direction = beefast_message_parser::kBackwardAndLeft;
            msg.distance  = 0.12;
            msg.angle = 20; 
            RCLCPP_INFO(rclcpp::get_logger("message_processor"),"只有右传感器有数据，向左转！");
        } else {  
            // 两个传感器都没有数据
            msg.direction = beefast_message_parser::kForward;
            RCLCPP_INFO(rclcpp::get_logger("message_processor"),"两个传感器都没有数据，碰撞状态退出！");
            return ;  
        }
        collision_publisher_->publish(msg);
        RCLCPP_INFO(rclcpp::get_logger("message_processor"), "process alarm done");
  }

 }// namespace beefast_message_parser