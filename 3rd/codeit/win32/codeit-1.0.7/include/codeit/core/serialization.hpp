#ifndef CODEIT_CORE_SERIALIZATION_H_
#define CODEIT_CORE_SERIALIZATION_H_

#include <string>
#include <filesystem>

#include <codeit/core/reflection.hpp>

namespace codeit::core
{
	auto toXmlString(codeit::core::Instance ins)->std::string;
	auto fromXmlString(codeit::core::Instance ins, std::string_view xml_str)->void;

	auto toXmlFile(codeit::core::Instance ins, const std::filesystem::path& file)->void;
	auto fromXmlFile(codeit::core::Instance ins, const std::filesystem::path& file)->void;

	//auto toJsonString(codeit::core::Instance ins)->std::string;
	//auto fromJsonString(codeit::core::Instance ins, std::string_view xml_str)->void;

}

#endif
