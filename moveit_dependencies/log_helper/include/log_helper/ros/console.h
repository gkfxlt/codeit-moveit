
#pragma once

/**
 * ROS_DEBUG
 */
/// ROS_DEBUG
#define ROS_DEBUG(...)                                                                                                 \
  do                                                                                                                   \
  {                                                                                                                    \
    logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_DEBUG) << format(__VA_ARGS__) << std::endl;                        \
  } while (false)
#define ROS_DEBUG_STREAM(args)                                                                                         \
  do                                                                                                                   \
  {                                                                                                                    \
    logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_DEBUG) << args << std::endl;                                       \
  } while (false)
#define ROS_DEBUG_NAMED(name, ...)                                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_DEBUG) << name << ": " << format(__VA_ARGS__) << std::endl;        \
  } while (false)
#define ROS_DEBUG_STREAM_NAMED(name, args)                                                                             \
  do                                                                                                                   \
  {                                                                                                                    \
    logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_DEBUG) << name << ": " << args << std::endl;                       \
  } while (false)

/// ROS_DEBUG_COND
#define ROS_DEBUG_COND(cond, ...)                                                                                      \
  do                                                                                                                   \
  {                                                                                                                    \
    if (cond)                                                                                                          \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_DEBUG) << format(__VA_ARGS__) << std::endl;                      \
  } while (false)
#define ROS_DEBUG_STREAM_COND(cond, args)                                                                              \
  do                                                                                                                   \
  {                                                                                                                    \
    if (cond)                                                                                                          \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_DEBUG) << args << std::endl;                                     \
  } while (false)
#define ROS_DEBUG_COND_NAMED(cond, name, ...)                                                                          \
  do                                                                                                                   \
  {                                                                                                                    \
    if (cond)                                                                                                          \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_DEBUG) << name << ": " << format(__VA_ARGS__) << std::endl;      \
  } while (false)
#define ROS_DEBUG_STREAM_COND_NAMED(cond, name, args)                                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    if (cond)                                                                                                          \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_DEBUG) << name << ": " << args << std::endl;                     \
  } while (false)

/// ROS_DEBUG_ONCE
#define ROS_DEBUG_ONCE(...)                                                                                            \
  do                                                                                                                   \
  {                                                                                                                    \
    static bool hit = false;                                                                                           \
    if (!hit)                                                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_DEBUG) << format(__VA_ARGS__) << std::endl;                      \
      hit = true;                                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_DEBUG_STREAM_ONCE(args)                                                                                    \
  do                                                                                                                   \
  {                                                                                                                    \
    static bool hit = false;                                                                                           \
    if (!hit)                                                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_DEBUG) << args << std::endl;                                     \
      hit = true;                                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_DEBUG_ONCE_NAMED(name, ...)                                                                                \
  do                                                                                                                   \
  {                                                                                                                    \
    static bool hit = false;                                                                                           \
    if (!hit)                                                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_DEBUG) << name << ": " << format(__VA_ARGS__) << std::endl;      \
      hit = true;                                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_DEBUG_STREAM_ONCE_NAMED(name, args)                                                                        \
  do                                                                                                                   \
  {                                                                                                                    \
    static bool hit = false;                                                                                           \
    if (!hit)                                                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_DEBUG) << name << ": " << args << std::endl;                     \
      hit = true;                                                                                                      \
    }                                                                                                                  \
  } while (false)

/// ROS_DEBUG_THROTTLE
#define ROS_DEBUG_THROTTLE(period, ...)                                                                                \
  do                                                                                                                   \
  {                                                                                                                    \
    static double last_log_sec = 0.0;                                                                                  \
    double now_log_sec = ros::Time::now().toSec();                                                                     \
    if (last_log_sec + period <= now_log_sec)                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_DEBUG) << format(__VA_ARGS__) << std::endl;                      \
      last_log_sec = now_log_sec;                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_DEBUG_STREAM_THROTTLE(period, args)                                                                        \
  do                                                                                                                   \
  {                                                                                                                    \
    static double last_log_sec = 0.0;                                                                                  \
    double now_log_sec = ros::Time::now().toSec();                                                                     \
    if (last_log_sec + period <= now_log_sec)                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_DEBUG) << args << std::endl;                                     \
      last_log_sec = now_log_sec;                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_DEBUG_THROTTLE_NAMED(period, name, ...)                                                                    \
  do                                                                                                                   \
  {                                                                                                                    \
    static double last_log_sec = 0.0;                                                                                  \
    double now_log_sec = ros::Time::now().toSec();                                                                     \
    if (last_log_sec + period <= now_log_sec)                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_DEBUG) << name << ": " << format(__VA_ARGS__) << std::endl;      \
      last_log_sec = now_log_sec;                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_DEBUG_STREAM_THROTTLE_NAMED(period, name, args)                                                            \
  do                                                                                                                   \
  {                                                                                                                    \
    static double last_log_sec = 0.0;                                                                                  \
    double now_log_sec = ros::Time::now().toSec();                                                                     \
    if (last_log_sec + period <= now_log_sec)                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_DEBUG) << name << ": " << args << std::endl;                     \
      last_log_sec = now_log_sec;                                                                                      \
    }                                                                                                                  \
  } while (false)

#define ROS_DEBUG_DELAYED_THROTTLE(period, ...)
#define ROS_DEBUG_STREAM_DELAYED_THROTTLE(period, args)
#define ROS_DEBUG_DELAYED_THROTTLE_NAMED(period, name, ...)
#define ROS_DEBUG_STREAM_DELAYED_THROTTLE_NAMED(period, name, args)

#define ROS_DEBUG_FILTER(filter, ...)
#define ROS_DEBUG_STREAM_FILTER(filter, args)
#define ROS_DEBUG_FILTER_NAMED(filter, name, ...)
#define ROS_DEBUG_STREAM_FILTER_NAMED(filter, name, args)

/**
 * ROS_INFO
 */
/// ROS_INFO
#define ROS_INFO(...)                                                                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_INFO) << format(__VA_ARGS__) << std::endl;                         \
  } while (false)
#define ROS_INFO_STREAM(args)                                                                                          \
  do                                                                                                                   \
  {                                                                                                                    \
    logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_INFO) << args << std::endl;                                        \
  } while (false)
#define ROS_INFO_NAMED(name, ...)                                                                                      \
  do                                                                                                                   \
  {                                                                                                                    \
    logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_INFO) << name << ": " << format(__VA_ARGS__) << std::endl;         \
  } while (false)
#define ROS_INFO_STREAM_NAMED(name, args)                                                                              \
  do                                                                                                                   \
  {                                                                                                                    \
    logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_INFO) << name << ": " << args << std::endl;                        \
  } while (false)

/// ROS_INFO_COND
#define ROS_INFO_COND(cond, ...)                                                                                       \
  do                                                                                                                   \
  {                                                                                                                    \
    if (cond)                                                                                                          \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_INFO) << format(__VA_ARGS__) << std::endl;                       \
  } while (false)
#define ROS_INFO_STREAM_COND(cond, args)                                                                               \
  do                                                                                                                   \
  {                                                                                                                    \
    if (cond)                                                                                                          \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_INFO) << args << std::endl;                                      \
  } while (false)
#define ROS_INFO_COND_NAMED(cond, name, ...)                                                                           \
  do                                                                                                                   \
  {                                                                                                                    \
    if (cond)                                                                                                          \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_INFO) << name << ": " << format(__VA_ARGS__) << std::endl;       \
  } while (false)
#define ROS_INFO_STREAM_COND_NAMED(cond, name, args)                                                                   \
  do                                                                                                                   \
  {                                                                                                                    \
    if (cond)                                                                                                          \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_INFO) << name << ": " << args << std::endl;                      \
  } while (false)

/// ROS_INFO_ONCE
#define ROS_INFO_ONCE(...)                                                                                             \
  do                                                                                                                   \
  {                                                                                                                    \
    static bool hit = false;                                                                                           \
    if (!hit)                                                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_INFO) << format(__VA_ARGS__) << std::endl;                       \
      hit = true;                                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_INFO_STREAM_ONCE(args)                                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    static bool hit = false;                                                                                           \
    if (!hit)                                                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_INFO) << args << std::endl;                                      \
      hit = true;                                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_INFO_ONCE_NAMED(name, ...)                                                                                 \
  do                                                                                                                   \
  {                                                                                                                    \
    static bool hit = false;                                                                                           \
    if (!hit)                                                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_INFO) << name << ": " << format(__VA_ARGS__) << std::endl;       \
      hit = true;                                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_INFO_STREAM_ONCE_NAMED(name, args)                                                                         \
  do                                                                                                                   \
  {                                                                                                                    \
    static bool hit = false;                                                                                           \
    if (!hit)                                                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_INFO) << name << ": " << args << std::endl;                      \
      hit = true;                                                                                                      \
    }                                                                                                                  \
  } while (false)

/// ROS_INFO_THROTTLE
#define ROS_INFO_THROTTLE(period, ...)                                                                                 \
  do                                                                                                                   \
  {                                                                                                                    \
    static double last_log_sec = 0.0;                                                                                  \
    double now_log_sec = ros::Time::now().toSec();                                                                     \
    if (last_log_sec + period <= now_log_sec)                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_INFO) << format(__VA_ARGS__) << std::endl;                       \
      last_log_sec = now_log_sec;                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_INFO_STREAM_THROTTLE(period, args)                                                                         \
  do                                                                                                                   \
  {                                                                                                                    \
    static double last_log_sec = 0.0;                                                                                  \
    double now_log_sec = ros::Time::now().toSec();                                                                     \
    if (last_log_sec + period <= now_log_sec)                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_INFO) << args << std::endl;                                      \
      last_log_sec = now_log_sec;                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_INFO_THROTTLE_NAMED(period, name, ...)                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    static double last_log_sec = 0.0;                                                                                  \
    double now_log_sec = ros::Time::now().toSec();                                                                     \
    if (last_log_sec + period <= now_log_sec)                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_INFO) << name << ": " << format(__VA_ARGS__) << std::endl;       \
      last_log_sec = now_log_sec;                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_INFO_STREAM_THROTTLE_NAMED(period, name, args)                                                             \
  do                                                                                                                   \
  {                                                                                                                    \
    static double last_log_sec = 0.0;                                                                                  \
    double now_log_sec = ros::Time::now().toSec();                                                                     \
    if (last_log_sec + period <= now_log_sec)                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_INFO) << name << ": " << args << std::endl;                      \
      last_log_sec = now_log_sec;                                                                                      \
    }                                                                                                                  \
  } while (false)

#define ROS_INFO_DELAYED_THROTTLE(period, ...)
#define ROS_INFO_STREAM_DELAYED_THROTTLE(period, args)
#define ROS_INFO_DELAYED_THROTTLE_NAMED(period, name, ...)
#define ROS_INFO_STREAM_DELAYED_THROTTLE_NAMED(period, name, args)

#define ROS_INFO_FILTER(filter, ...)
#define ROS_INFO_STREAM_FILTER(filter, args)
#define ROS_INFO_FILTER_NAMED(filter, name, ...)
#define ROS_INFO_STREAM_FILTER_NAMED(filter, name, args)

/**
 * ROS_WARN
 */
/// ROS_WARN
#define ROS_WARN(...)                                                                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_WARN) << format(__VA_ARGS__) << std::endl;                         \
  } while (false)
#define ROS_WARN_STREAM(args)                                                                                          \
  do                                                                                                                   \
  {                                                                                                                    \
    logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_WARN) << args << std::endl;                                        \
  } while (false)
#define ROS_WARN_NAMED(name, ...)                                                                                      \
  do                                                                                                                   \
  {                                                                                                                    \
    logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_WARN) << name << ": " << format(__VA_ARGS__) << std::endl;         \
  } while (false)
#define ROS_WARN_STREAM_NAMED(name, args)                                                                              \
  do                                                                                                                   \
  {                                                                                                                    \
    logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_WARN) << name << ": " << args << std::endl;                        \
  } while (false)

/// ROS_WARN_COND
#define ROS_WARN_COND(cond, ...)                                                                                       \
  do                                                                                                                   \
  {                                                                                                                    \
    if (cond)                                                                                                          \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_WARN) << format(__VA_ARGS__) << std::endl;                       \
  } while (false)
#define ROS_WARN_STREAM_COND(cond, args)                                                                               \
  do                                                                                                                   \
  {                                                                                                                    \
    if (cond)                                                                                                          \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_WARN) << args << std::endl;                                      \
  } while (false)
#define ROS_WARN_COND_NAMED(cond, name, ...)                                                                           \
  do                                                                                                                   \
  {                                                                                                                    \
    if (cond)                                                                                                          \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_WARN) << name << ": " << format(__VA_ARGS__) << std::endl;       \
  } while (false)
#define ROS_WARN_STREAM_COND_NAMED(cond, name, args)                                                                   \
  do                                                                                                                   \
  {                                                                                                                    \
    if (cond)                                                                                                          \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_WARN) << name << ": " << args << std::endl;                      \
  } while (false)

/// ROS_WARN_ONCE
#define ROS_WARN_ONCE(...)                                                                                             \
  do                                                                                                                   \
  {                                                                                                                    \
    static bool hit = false;                                                                                           \
    if (!hit)                                                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_WARN) << format(__VA_ARGS__) << std::endl;                       \
      hit = true;                                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_WARN_STREAM_ONCE(args)                                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    static bool hit = false;                                                                                           \
    if (!hit)                                                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_WARN) << args << std::endl;                                      \
      hit = true;                                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_WARN_ONCE_NAMED(name, ...)                                                                                 \
  do                                                                                                                   \
  {                                                                                                                    \
    static bool hit = false;                                                                                           \
    if (!hit)                                                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_WARN) << name << ": " << format(__VA_ARGS__) << std::endl;       \
      hit = true;                                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_WARN_STREAM_ONCE_NAMED(name, args)                                                                         \
  do                                                                                                                   \
  {                                                                                                                    \
    static bool hit = false;                                                                                           \
    if (!hit)                                                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_WARN) << name << ": " << args << std::endl;                      \
      hit = true;                                                                                                      \
    }                                                                                                                  \
  } while (false)

/// ROS_WARN_THROTTLE
#define ROS_WARN_THROTTLE(period, ...)                                                                                 \
  do                                                                                                                   \
  {                                                                                                                    \
    static double last_log_sec = 0.0;                                                                                  \
    double now_log_sec = ros::Time::now().toSec();                                                                     \
    if (last_log_sec + period <= now_log_sec)                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_WARN) << format(__VA_ARGS__) << std::endl;                       \
      last_log_sec = now_log_sec;                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_WARN_STREAM_THROTTLE(period, args)                                                                         \
  do                                                                                                                   \
  {                                                                                                                    \
    static double last_log_sec = 0.0;                                                                                  \
    double now_log_sec = ros::Time::now().toSec();                                                                     \
    if (last_log_sec + period <= now_log_sec)                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_WARN) << args << std::endl;                                      \
      last_log_sec = now_log_sec;                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_WARN_THROTTLE_NAMED(period, name, ...)                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    static double last_log_sec = 0.0;                                                                                  \
    double now_log_sec = ros::Time::now().toSec();                                                                     \
    if (last_log_sec + period <= now_log_sec)                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_WARN) << name << ": " << format(__VA_ARGS__) << std::endl;       \
      last_log_sec = now_log_sec;                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_WARN_STREAM_THROTTLE_NAMED(period, name, args)                                                             \
  do                                                                                                                   \
  {                                                                                                                    \
    static double last_log_sec = 0.0;                                                                                  \
    double now_log_sec = ros::Time::now().toSec();                                                                     \
    if (last_log_sec + period <= now_log_sec)                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_WARN) << name << ": " << args << std::endl;                      \
      last_log_sec = now_log_sec;                                                                                      \
    }                                                                                                                  \
  } while (false)

#define ROS_WARN_DELAYED_THROTTLE(period, ...)
#define ROS_WARN_STREAM_DELAYED_THROTTLE(period, args)
#define ROS_WARN_DELAYED_THROTTLE_NAMED(period, name, ...)
#define ROS_WARN_STREAM_DELAYED_THROTTLE_NAMED(period, name, args)

#define ROS_WARN_FILTER(filter, ...)
#define ROS_WARN_STREAM_FILTER(filter, args)
#define ROS_WARN_FILTER_NAMED(filter, name, ...)
#define ROS_WARN_STREAM_FILTER_NAMED(filter, name, args)

/**
 * ROS_ERROR
 */
/// ROS_ERROR
#define ROS_ERROR(...)                                                                                                 \
  do                                                                                                                   \
  {                                                                                                                    \
    logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_ERROR) << format(__VA_ARGS__) << std::endl;                        \
  } while (false)
#define ROS_ERROR_STREAM(args)                                                                                         \
  do                                                                                                                   \
  {                                                                                                                    \
    logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_ERROR) << args << std::endl;                                       \
  } while (false)
#define ROS_ERROR_NAMED(name, ...)                                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_ERROR) << name << ": " << format(__VA_ARGS__) << std::endl;        \
  } while (false)
#define ROS_ERROR_STREAM_NAMED(name, args)                                                                             \
  do                                                                                                                   \
  {                                                                                                                    \
    logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_ERROR) << name << ": " << args << std::endl;                       \
  } while (false)

/// ROS_ERROR_COND
#define ROS_ERROR_COND(cond, ...)                                                                                      \
  do                                                                                                                   \
  {                                                                                                                    \
    if (cond)                                                                                                          \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_ERROR) << format(__VA_ARGS__) << std::endl;                      \
  } while (false)
#define ROS_ERROR_STREAM_COND(cond, args)                                                                              \
  do                                                                                                                   \
  {                                                                                                                    \
    if (cond)                                                                                                          \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_ERROR) << args << std::endl;                                     \
  } while (false)
#define ROS_ERROR_COND_NAMED(cond, name, ...)                                                                          \
  do                                                                                                                   \
  {                                                                                                                    \
    if (cond)                                                                                                          \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_ERROR) << name << ": " << format(__VA_ARGS__) << std::endl;      \
  } while (false)
#define ROS_ERROR_STREAM_COND_NAMED(cond, name, args)                                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    if (cond)                                                                                                          \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_ERROR) << name << ": " << args << std::endl;                     \
  } while (false)

/// ROS_ERROR_ONCE
#define ROS_ERROR_ONCE(...)                                                                                            \
  do                                                                                                                   \
  {                                                                                                                    \
    static bool hit = false;                                                                                           \
    if (!hit)                                                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_ERROR) << format(__VA_ARGS__) << std::endl;                      \
      hit = true;                                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_ERROR_STREAM_ONCE(args)                                                                                    \
  do                                                                                                                   \
  {                                                                                                                    \
    static bool hit = false;                                                                                           \
    if (!hit)                                                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_ERROR) << args << std::endl;                                     \
      hit = true;                                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_ERROR_ONCE_NAMED(name, ...)                                                                                \
  do                                                                                                                   \
  {                                                                                                                    \
    static bool hit = false;                                                                                           \
    if (!hit)                                                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_ERROR) << name << ": " << format(__VA_ARGS__) << std::endl;      \
      hit = true;                                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_ERROR_STREAM_ONCE_NAMED(name, args)                                                                        \
  do                                                                                                                   \
  {                                                                                                                    \
    static bool hit = false;                                                                                           \
    if (!hit)                                                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_ERROR) << name << ": " << args << std::endl;                     \
      hit = true;                                                                                                      \
    }                                                                                                                  \
  } while (false)

/// ROS_ERROR_THROTTLE
#define ROS_ERROR_THROTTLE(period, ...)                                                                                \
  do                                                                                                                   \
  {                                                                                                                    \
    static double last_log_sec = 0.0;                                                                                  \
    double now_log_sec = ros::Time::now().toSec();                                                                     \
    if (last_log_sec + period <= now_log_sec)                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_ERROR) << format(__VA_ARGS__) << std::endl;                      \
      last_log_sec = now_log_sec;                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_ERROR_STREAM_THROTTLE(period, args)                                                                        \
  do                                                                                                                   \
  {                                                                                                                    \
    static double last_log_sec = 0.0;                                                                                  \
    double now_log_sec = ros::Time::now().toSec();                                                                     \
    if (last_log_sec + period <= now_log_sec)                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_ERROR) << args << std::endl;                                     \
      last_log_sec = now_log_sec;                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_ERROR_THROTTLE_NAMED(period, name, ...)                                                                    \
  do                                                                                                                   \
  {                                                                                                                    \
    static double last_log_sec = 0.0;                                                                                  \
    double now_log_sec = ros::Time::now().toSec();                                                                     \
    if (last_log_sec + period <= now_log_sec)                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_ERROR) << name << ": " << format(__VA_ARGS__) << std::endl;      \
      last_log_sec = now_log_sec;                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_ERROR_STREAM_THROTTLE_NAMED(period, name, args)                                                            \
  do                                                                                                                   \
  {                                                                                                                    \
    static double last_log_sec = 0.0;                                                                                  \
    double now_log_sec = ros::Time::now().toSec();                                                                     \
    if (last_log_sec + period <= now_log_sec)                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_ERROR) << name << ": " << args << std::endl;                     \
      last_log_sec = now_log_sec;                                                                                      \
    }                                                                                                                  \
  } while (false)

#define ROS_ERROR_DELAYED_THROTTLE(period, ...)
#define ROS_ERROR_STREAM_DELAYED_THROTTLE(period, args)
#define ROS_ERROR_DELAYED_THROTTLE_NAMED(period, name, ...)
#define ROS_ERROR_STREAM_DELAYED_THROTTLE_NAMED(period, name, args)

#define ROS_ERROR_FILTER(filter, ...)
#define ROS_ERROR_STREAM_FILTER(filter, args)
#define ROS_ERROR_FILTER_NAMED(filter, name, ...)
#define ROS_ERROR_STREAM_FILTER_NAMED(filter, name, args)

/**
 * ROS_FATAL
 */
/// ROS_FATAL
#define ROS_FATAL(...)                                                                                                 \
  do                                                                                                                   \
  {                                                                                                                    \
    logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_FATAL) << format(__VA_ARGS__) << std::endl;                        \
  } while (false)
#define ROS_FATAL_STREAM(args)                                                                                         \
  do                                                                                                                   \
  {                                                                                                                    \
    logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_FATAL) << args << std::endl;                                       \
  } while (false)
#define ROS_FATAL_NAMED(name, ...)                                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_FATAL) << name << ": " << format(__VA_ARGS__) << std::endl;        \
  } while (false)
#define ROS_FATAL_STREAM_NAMED(name, args)                                                                             \
  do                                                                                                                   \
  {                                                                                                                    \
    logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_FATAL) << name << ": " << args << std::endl;                       \
  } while (false)

/// ROS_FATAL_COND
#define ROS_FATAL_COND(cond, ...)                                                                                      \
  do                                                                                                                   \
  {                                                                                                                    \
    if (cond)                                                                                                          \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_FATAL) << format(__VA_ARGS__) << std::endl;                      \
  } while (false)
#define ROS_FATAL_STREAM_COND(cond, args)                                                                              \
  do                                                                                                                   \
  {                                                                                                                    \
    if (cond)                                                                                                          \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_FATAL) << args << std::endl;                                     \
  } while (false)
#define ROS_FATAL_COND_NAMED(cond, name, ...)                                                                          \
  do                                                                                                                   \
  {                                                                                                                    \
    if (cond)                                                                                                          \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_FATAL) << name << ": " << format(__VA_ARGS__) << std::endl;      \
  } while (false)
#define ROS_FATAL_STREAM_COND_NAMED(cond, name, args)                                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    if (cond)                                                                                                          \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_FATAL) << name << ": " << args << std::endl;                     \
  } while (false)

/// ROS_FATAL_ONCE
#define ROS_FATAL_ONCE(...)                                                                                            \
  do                                                                                                                   \
  {                                                                                                                    \
    static bool hit = false;                                                                                           \
    if (!hit)                                                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_FATAL) << format(__VA_ARGS__) << std::endl;                      \
      hit = true;                                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_FATAL_STREAM_ONCE(args)                                                                                    \
  do                                                                                                                   \
  {                                                                                                                    \
    static bool hit = false;                                                                                           \
    if (!hit)                                                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_FATAL) << args << std::endl;                                     \
      hit = true;                                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_FATAL_ONCE_NAMED(name, ...)                                                                                \
  do                                                                                                                   \
  {                                                                                                                    \
    static bool hit = false;                                                                                           \
    if (!hit)                                                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_FATAL) << name << ": " << format(__VA_ARGS__) << std::endl;      \
      hit = true;                                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_FATAL_STREAM_ONCE_NAMED(name, args)                                                                        \
  do                                                                                                                   \
  {                                                                                                                    \
    static bool hit = false;                                                                                           \
    if (!hit)                                                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_FATAL) << name << ": " << args << std::endl;                     \
      hit = true;                                                                                                      \
    }                                                                                                                  \
  } while (false)

/// ROS_FATAL_THROTTLE
#define ROS_FATAL_THROTTLE(period, ...)                                                                                \
  do                                                                                                                   \
  {                                                                                                                    \
    static double last_log_sec = 0.0;                                                                                  \
    double now_log_sec = ros::Time::now().toSec();                                                                     \
    if (last_log_sec + period <= now_log_sec)                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_FATAL) << format(__VA_ARGS__) << std::endl;                      \
      last_log_sec = now_log_sec;                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_FATAL_STREAM_THROTTLE(period, args)                                                                        \
  do                                                                                                                   \
  {                                                                                                                    \
    static double last_log_sec = 0.0;                                                                                  \
    double now_log_sec = ros::Time::now().toSec();                                                                     \
    if (last_log_sec + period <= now_log_sec)                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_FATAL) << args << std::endl;                                     \
      last_log_sec = now_log_sec;                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_FATAL_THROTTLE_NAMED(period, name, ...)                                                                    \
  do                                                                                                                   \
  {                                                                                                                    \
    static double last_log_sec = 0.0;                                                                                  \
    double now_log_sec = ros::Time::now().toSec();                                                                     \
    if (last_log_sec + period <= now_log_sec)                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_FATAL) << name << ": " << format(__VA_ARGS__) << std::endl;      \
      last_log_sec = now_log_sec;                                                                                      \
    }                                                                                                                  \
  } while (false)
#define ROS_FATAL_STREAM_THROTTLE_NAMED(period, name, args)                                                            \
  do                                                                                                                   \
  {                                                                                                                    \
    static double last_log_sec = 0.0;                                                                                  \
    double now_log_sec = ros::Time::now().toSec();                                                                     \
    if (last_log_sec + period <= now_log_sec)                                                                          \
    {                                                                                                                  \
      logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_FATAL) << name << ": " << args << std::endl;                     \
      last_log_sec = now_log_sec;                                                                                      \
    }                                                                                                                  \
  } while (false)

#define ROS_FATAL_DELAYED_THROTTLE(period, ...)
#define ROS_FATAL_STREAM_DELAYED_THROTTLE(period, args)
#define ROS_FATAL_DELAYED_THROTTLE_NAMED(period, name, ...)
#define ROS_FATAL_STREAM_DELAYED_THROTTLE_NAMED(period, name, args)

#define ROS_FATAL_FILTER(filter, ...)
#define ROS_FATAL_STREAM_FILTER(filter, args)
#define ROS_FATAL_FILTER_NAMED(filter, name, ...)
#define ROS_FATAL_STREAM_FILTER_NAMED(filter, name, args)
