
#pragma once

#include <urdf_parser/urdf_parser.h>

namespace urdf 
{
	class URDFXMLParser final : public urdf::URDFParser
	{
	public:
		URDFXMLParser() = default;

		~URDFXMLParser() = default;

		urdf::ModelInterfaceSharedPtr parse(const std::string & xml_string) override;

		size_t might_handle(const std::string & data) override;
	};
}

