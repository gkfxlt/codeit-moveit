//
// Created by Administrator on 2021/6/23.
//

#pragma once

#include <string>

namespace stdboost
{
template <typename T>
inline T lexical_cast(const std::string& s)
{
  return s;
};

template <>
inline bool lexical_cast<bool>(const std::string& s)
{
  std::string str = s;
  std::transform(str.begin(), str.end(), str.begin(), ::tolower);
  if (str.empty() || (str == "false") || (str == "0"))
  {
    return false;
  }
  return true;
};

template <>
inline int lexical_cast<int>(const std::string& s)
{
  return std::stoi(s);
};

template <>
inline float lexical_cast<float>(const std::string& s)
{
  return std::stof(s);
};

template <>
inline double lexical_cast<double>(const std::string& s)
{
  return std::stod(s);
};

template <typename T>
inline T lexical_cast(const char* s)
{
  return lexical_cast<T>(std::string(s));
};
}  // namespace stdboost
