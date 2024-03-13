//
// Created by fan on 2021/8/9.
//

#pragma once
#include <iostream>
#include <string>
#include "log_helper/visibility_control.hpp"
#include <stdarg.h>
#include "codeit/core/log.hpp"
#include<sstream>
enum class LogLevel
{
  LOG_LEVEL_DEBUG = 0,
  LOG_LEVEL_INFO,
  LOG_LEVEL_WARN,
  LOG_LEVEL_ERROR,
  LOG_LEVEL_FATAL,
  LOG_LEVEL_NONE
};

//LOG_HELPER_PUBLIC
/*__attribute__((weak))*/static std::ostream& logHead(const std::string& _file_, int _line_, LogLevel level) {
    static thread_local std::stringstream ss;
    return ss;
}

//LOG_HELPER_PUBLIC
/*__attribute__((weak))*/static std::string format(const char* fmt, ...)
{
  char buf[1024];

  va_list ap;
  va_start(ap, fmt);
#ifdef _MSC_VER
  vsnprintf_s(buf, sizeof(buf), _TRUNCATE, fmt, ap);
#else
  vsnprintf(buf, sizeof(buf), fmt, ap);
#endif
  va_end(ap);

  return std::string(buf);
}


#include "log_helper/ros/console.h"
#include "log_helper/console_bridge/console.h"
