
#include "log_helper/log_helper.h"
#include <sstream>

/*
#if USE_STD_OUT
#include <iomanip>
#include <chrono>
#include <iostream>
std::string get_date_time()
{
  auto to_string = [](const std::chrono::system_clock::time_point& t) -> std::string {
    auto as_time_t = std::chrono::system_clock::to_time_t(t);
    struct tm tm;
#if defined(WIN32) || defined(_WINDLL)
    localtime_s(&tm, &as_time_t);  // win api，线程安全，而std::localtime线程不安全
#else
    localtime_r(&as_time_t, &tm);  // linux api，线程安全
#endif
    std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch());
    char buf[128];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d %03lld", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
             tm.tm_hour, tm.tm_min, tm.tm_sec, ms.count() % 1000);
    return buf;
  };

  std::chrono::system_clock::time_point t = std::chrono::system_clock::now();
  return to_string(t);
}

std::ostream& logHead(const std::string& _file_, int _line_, LogLevel level)
{
  std::stringstream ss;
  ss << " | " << get_date_time() << " | "
     << _file_.substr(_file_.find_last_of("/\\") + 1) << ":" << _line_ << " | ";

  switch (level)
  {
    case LogLevel::LOG_LEVEL_DEBUG:
      return std::cout << std::setw(5) << "DEBUG" << ss.str();

    case LogLevel::LOG_LEVEL_INFO:
      return std::cout << std::setw(5) << "INFO" << ss.str();

    case LogLevel::LOG_LEVEL_WARN:
      return std::cout << std::setw(5) << "WARN" << ss.str();

    case LogLevel::LOG_LEVEL_ERROR:
      return std::cerr << std::setw(5) << "ERROR" << ss.str();

    case LogLevel::LOG_LEVEL_FATAL:
      return std::cerr << std::setw(5) << "FATAL" << ss.str();

    default:
      return std::cerr;
  }
}
#else
*/
#include "codeit/core/log.hpp"
std::ostream& logHead(const std::string& _file_, int _line_, LogLevel level)
{
  using namespace codeit;

  std::stringstream ss;

  return ss;
  /*
  ss << "|" << std::setw(core::LOG_TIME_WIDTH) << core::logFileTimeFormat(std::chrono::system_clock::now()) << "|"
     << std::setw(core::LOG_FILE_WIDTH) << _file_.substr(_file_.find_last_of("/\\") + 1) << "|"
     << std::setw(core::LOG_LINE_WIDTH) << _line_ << "|";

  switch (level)
  {
    case LogLevel::LOG_LEVEL_DEBUG:
      return core::log() << std::setw(core::LOG_TYPE_WIDTH) << "DEBUG" << ss.str();

    case LogLevel::LOG_LEVEL_INFO:
      return core::log() << std::setw(core::LOG_TYPE_WIDTH) << "INFO" << ss.str();

    case LogLevel::LOG_LEVEL_WARN:
      return core::log() << std::setw(core::LOG_TYPE_WIDTH) << "WARN" << ss.str();

    case LogLevel::LOG_LEVEL_ERROR:
      return core::log() << std::setw(core::LOG_TYPE_WIDTH) << "ERROR" << ss.str();

    case LogLevel::LOG_LEVEL_FATAL:
      return core::log() << std::setw(core::LOG_TYPE_WIDTH) << "FATAL" << ss.str();

    default:
      return LOG_COUT;
  }
  */
}
/*
#endif
*/
#include <stdarg.h>
std::string format(const char* fmt, ...)
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
