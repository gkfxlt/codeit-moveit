#ifndef CODEIT_LOG_H_
#define CODEIT_LOG_H_
#include <string>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <filesystem>
#include <ctime>
#include <exception>
#include<iomanip>
#include<iostream>
#include<fstream>
#include <mutex>

#define LOG_COUT core::cout()
//void LogFile(std::string msgs);

#define LOG_DEBUG core::log()\
	<< std::setw(core::LOG_TYPE_WIDTH) << "DEBUG" << "|" \
	<< std::setw(core::LOG_TIME_WIDTH) << core::logFileTimeFormat(std::chrono::system_clock::now()) <<"|" \
	<< std::setw(core::LOG_FILE_WIDTH) << std::string(__FILE__).substr(std::string(__FILE__).find_last_of("/\\") + 1) <<"|"\
	<< std::setw(core::LOG_LINE_WIDTH) << __LINE__ <<"|"

#define LOG_INFO  core::log()\
	<< std::setw(core::LOG_TYPE_WIDTH) << "INFO" << "|" \
	<< std::setw(core::LOG_TIME_WIDTH) << core::logFileTimeFormat(std::chrono::system_clock::now()) <<"|" \
	<< std::setw(core::LOG_FILE_WIDTH) << std::string(__FILE__).substr(std::string(__FILE__).find_last_of("/\\") + 1) <<"|"\
	<< std::setw(core::LOG_LINE_WIDTH) << __LINE__ <<"|"

#define LOG_WARN   core::log()\
	<< std::setw(core::LOG_TYPE_WIDTH) << "WARN" << "|" \
	<< std::setw(core::LOG_TIME_WIDTH) << core::logFileTimeFormat(std::chrono::system_clock::now()) <<"|" \
	<< std::setw(core::LOG_FILE_WIDTH) << std::string(__FILE__).substr(std::string(__FILE__).find_last_of("/\\") + 1) <<"|"\
	<< std::setw(core::LOG_LINE_WIDTH) << __LINE__ <<"|"

#define LOG_ERROR core::log()\
	<< std::setw(core::LOG_TYPE_WIDTH) << "ERROR" << "|" \
	<< std::setw(core::LOG_TIME_WIDTH) << core::logFileTimeFormat(std::chrono::system_clock::now()) <<"|" \
	<< std::setw(core::LOG_FILE_WIDTH) << std::string(__FILE__).substr(std::string(__FILE__).find_last_of("/\\") + 1) <<"|"\
	<< std::setw(core::LOG_LINE_WIDTH) << __LINE__ <<"|"

#define LOG_FATAL core::log()\
	<< std::setw(core::LOG_TYPE_WIDTH) << "FATAL" << "|" \
	<< std::setw(core::LOG_TIME_WIDTH) << core::logFileTimeFormat(std::chrono::system_clock::now()) <<"|" \
	<< std::setw(core::LOG_FILE_WIDTH) << std::string(__FILE__).substr(std::string(__FILE__).find_last_of("/\\") + 1) <<"|"\
	<< std::setw(core::LOG_LINE_WIDTH) << __LINE__ <<"|"

#define THROW_FILE_LINE(error) LOG_ERROR<< std::string(__FILE__) + "_" + std::to_string(__LINE__) + ":" + error<<std::endl, throw std::runtime_error(error)

#define LOG_AND_THROW(error) LOG_ERROR << error.what() << std::endl, throw error

namespace codeit::core
{
	enum
	{
		LOG_TYPE_WIDTH = 5,
		LOG_TIME_WIDTH = 20,
		LOG_FILE_WIDTH = 25,
		LOG_LINE_WIDTH = 5,
		LOG_SPACE_WIDTH = LOG_TYPE_WIDTH + 1 + LOG_TIME_WIDTH + 1 + LOG_FILE_WIDTH + 1 + LOG_LINE_WIDTH + 1,
	};

	// ����log�ļ��У�����Ϊ��ʱ���������ļ�·����Ϊlog·�� //
	auto logDirectory(const std::filesystem::path& log_dir_path = std::filesystem::path())->void;
	// ����log�ļ�������������Ի����·����Ϊ��ʱ����Ĭ��ֵ //
	auto logFile(const std::filesystem::path& log_file_path = std::filesystem::path())->void;
	// ���õ���log�ļ�����������С��0ʱ������ //
	auto logMaxInfoNum(int max_info_num = 100000);
	// ����log stream //
	auto log()->std::ostream&;

	auto logDirPath()->std::filesystem::path;
	auto logExeName()->std::string;
	auto logFileTimeFormat(const std::chrono::system_clock::time_point& time)->std::string;

	auto cout()->std::ostream&;
}
#endif
