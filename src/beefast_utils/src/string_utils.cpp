// Copyright (c) 2024, Ontoptech Inc.
// All rights reserved.
// Author: Rockey Shao

#include "beefast_utils/string_utils.hpp"

using std::string;

namespace beefast_utils
{

std::string strip_leading_slash(const string & in)
{
  string out = in;

  if ((!in.empty()) && (in[0] == '/')) {
    out.erase(0, 1);
  }

  return out;
}

Tokens split(const string & tokenstring, const char delimiter)
{
  Tokens tokens;

  size_t current_pos = 0;
  size_t pos = 0;
  while ((pos = tokenstring.find(delimiter, current_pos)) != string::npos) {
    tokens.push_back(tokenstring.substr(current_pos, pos - current_pos));
    current_pos = pos + 1;
  }
  tokens.push_back(tokenstring.substr(current_pos));
  return tokens;
}

std::string toString(double val, int precision)
{
  std::ostringstream out;
  out.precision(precision);
  out << std::fixed << val;
  return out.str();
}

// 将rclcpp::Time转换为string格式的日期时间  
std::string format_time(const rclcpp::Time& time) {  
    // 将秒数和纳秒数转换为time_t（可能需要处理时间溢出）  
    auto seconds_since_epoch = time.seconds();  
    std::time_t t = static_cast<std::time_t>(seconds_since_epoch);  
  
    // 创建一个本地时间的tm结构体  
    std::tm* local_time = std::localtime(&t);  
  
    // 创建一个输出字符串流  
    std::ostringstream oss;  
    oss << std::put_time(local_time, "%Y-%m-%d %H:%M:%S");  
  
    // 添加纳秒部分  
    oss << "." << std::setfill('0') << std::setw(9) << time.nanoseconds();  
  
    return oss.str();  
} 

}  // namespace beefast_utils