#ifndef CODEIT_SYSTEM_ERRORINFO_H_
#define CODEIT_SYSTEM_ERRORINFO_H_

#include <codeit/core/core.hpp>

namespace codeit::system
{
	
	class ErrorInfoRoot : public core::Object
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		CODEIT_REGISTER_TYPE(ErrorInfoRoot);

	private:
		core::XmlDocument doc_;
	};
	class ErrorInfo :public core::Object
	{
	public:
		
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto infoCN() ->std::string&;
		auto infoEN() ->std::string&;
		auto info() -> std::string&;
		auto errLevel() const->const std::string&;
		auto errCode() const->const std::int64_t&;

		ErrorInfo(const std::string name = "error", \
			const std::int64_t errCode = -1, \
			const std::string errLevel = "ERROR", \
			const std::string infoCN="¥ÌŒÛ–≈œ¢",\
		    const std::string infoEN="Error Info");
		CODEIT_REGISTER_TYPE(ErrorInfo);
		CODEIT_DEFINE_BIG_FOUR(ErrorInfo);
	private:
		std::string errLevel_,infoCN_, infoEN_,ret_msg;
		std::int64_t errCode_;
	};

	auto defaultErrorInfoPool(core::ObjectPool<codeit::system::ErrorInfo>& errorinfoPool, std::map<std::int32_t, std::string>& errMap)->void;

}

#endif

