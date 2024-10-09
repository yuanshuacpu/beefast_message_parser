// Copyright (c) 2024, Ontoptech Inc.
// All rights reserved.
// Author: Rockey Shao

#ifndef BEEFAST_UTILS__STRING_UTILS_HPP_
#define BEEFAST_UTILS__STRING_UTILS_HPP_

#include <string>
#include <vector>
 #include <sstream>
 #include <ctime> 
 #include "rclcpp/rclcpp.hpp"

namespace beefast_utils
{

typedef std::vector<std::string> Tokens;

/*
 * @brief Remove leading slash from a topic name
 * @param in String of topic in
 * @return String out without slash
*/
std::string strip_leading_slash(const std::string & in);

///
/*
 * @brief Split a string at the delimiters
 * @param in String to split
 * @param Delimiter criteria
 * @return Tokens
*/
Tokens split(const std::string & tokenstring, const char delimiter);

std::string toString(double val, int precision);

std::string format_time(const rclcpp::Time& time);

}  // namespace beefast_utils

#endif  // BEEFAST_UTILS__STRING_UTILS_HPP_
