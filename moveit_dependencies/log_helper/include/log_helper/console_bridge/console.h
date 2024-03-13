
#pragma once

#define CONSOLE_BRIDGE_logDebug(...)                                                                                   \
  do                                                                                                                   \
  {                                                                                                                    \
    logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_DEBUG) << format(__VA_ARGS__) << std::endl;                   \
  } while (false)

#define CONSOLE_BRIDGE_logInform(...)                                                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_INFO) << format(__VA_ARGS__) << std::endl;                    \
  } while (false)

#define CONSOLE_BRIDGE_logWarn(...)                                                                                    \
  do                                                                                                                   \
  {                                                                                                                    \
    logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_WARN) << format(__VA_ARGS__) << std::endl;                    \
  } while (false)

#define CONSOLE_BRIDGE_logError(...)                                                                                   \
  do                                                                                                                   \
  {                                                                                                                    \
    logHead(__FILE__, __LINE__, LogLevel::LOG_LEVEL_ERROR) << format(__VA_ARGS__) << std::endl;                   \
  } while (false)
