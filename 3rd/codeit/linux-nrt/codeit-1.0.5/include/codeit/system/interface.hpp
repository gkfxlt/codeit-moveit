#ifndef CODEIT_SYSTEM_INTERFACE_H_
#define CODEIT_SYSTEM_INTERFACE_H_

#include <string>
#include <sstream>
#include <map>
#include <memory>
#include <future>

#include <codeit/core/core.hpp>
#include <codeit/controller/controller.hpp>
//#include <aris/sensor/sensor.hpp>
#include <codeit/model/model.hpp>
#include <codeit/function/basefunc.hpp>

#include<codeit/core/serialport.hpp>
namespace codeit::system
{
	class InterfaceRoot : public core::Object
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		CODEIT_REGISTER_TYPE(InterfaceRoot);

	private:
		core::XmlDocument doc_;
	};
	auto parse_recv_str(std::string str, std::vector<std::pair<std::string, std::any>>& ret)->void;
	auto parse_ret_value(std::vector<std::pair<std::string, std::any>>& ret)->std::string;
	auto multiLingual(codeit::function::BasisFunc& plan) ->void;
	class Interface :public core::Object
	{
	public:
		auto virtual open()->void = 0;
		auto virtual close()->void = 0;

		Interface(const std::string& name = "interface");
		CODEIT_REGISTER_TYPE(Interface);
		CODEIT_DEFINE_BIG_FOUR(Interface);
	};

	class ComInterface :public Interface
	{
	public:
		auto virtual open()->void override;
		auto virtual close()->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		ComInterface(const std::string& name = "com_interface", const core::SerialPort::ComOptions& options = core::SerialPort::defaultComOptions);
		CODEIT_REGISTER_TYPE(ComInterface);
		CODEIT_DEFINE_BIG_FOUR(ComInterface);

	private:
		core::SerialPort* com_;
	};

	class StateRtInterface :public Interface
	{
	public:
		auto virtual open()->void override;
		auto virtual close()->void override;
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		StateRtInterface(const std::string& name = "state_interface", const std::string& remote_ip = "", const std::string& port = "5866", core::Socket::TYPE type = core::Socket::TCP, const Size interval_ = 30);
		StateRtInterface(StateRtInterface&& other);
		StateRtInterface& operator=(StateRtInterface&& other);
		CODEIT_REGISTER_TYPE(StateRtInterface);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
		core::Socket* sock_;
	};

	class ProgramWebInterface :public Interface
	{
	public:
		auto virtual open()->void override;
		auto virtual close()->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto isAutoMode()->bool;
		auto isAutoRunning()->bool;
		auto isAutoPaused()->bool;
		auto isAutoStopped()->bool;
		auto currentLine()->int;
		auto lastError()->std::string;
		auto lastErrorCode()->int;
		auto lastErrorLine()->int;

		ProgramWebInterface(const std::string& name = "pro_interface", const std::string& remote_ip = "", const std::string& port = "5866", core::Socket::TYPE type = core::Socket::WEB);
		ProgramWebInterface(ProgramWebInterface&& other);
		ProgramWebInterface& operator=(ProgramWebInterface&& other);
		CODEIT_REGISTER_TYPE(ProgramWebInterface);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
		core::Socket* sock_;
	};

	auto parse_ret_value(std::vector<std::pair<std::string, std::any>>& ret)->std::string;
	class WebInterface :public Interface
	{
	public:
		auto virtual open()->void override;
		auto virtual close()->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		WebInterface(const std::string& name = "websock_interface", const std::string& remote_ip = "", const std::string& port = "5866", core::Socket::TYPE type = core::Socket::WEB, const bool& isServer=true);
		CODEIT_REGISTER_TYPE(WebInterface);
		CODEIT_DEFINE_BIG_FOUR(WebInterface);

	private:
		core::Socket* sock_;
	};
	/*class HttpInterface :public Interface
	{
	public:
		auto virtual open()->void override;
		auto virtual close()->void override;
		auto virtual saveXml(core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement &xml_ele)->void override;

		virtual ~HttpInterface();
		HttpInterface(const std::string &name = "http_interface", const std::string &port = "8000", const std::string &document_root = "./");
		HttpInterface(const HttpInterface & other) = delete;
		HttpInterface(HttpInterface && other);
		HttpInterface &operator=(const HttpInterface& other) = delete;
		HttpInterface &operator=(HttpInterface&& other);
		ARIS_REGISTER_TYPE(HttpInterface);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};*/

	//class GetInfo :public aris::cmdtarget::BasisFunc
	//{
	//public:
	//	auto virtual prepareNrt(cmdtarget::BasisFunc& plan_next,int)->void;
	//	//GetInfo() {	this->command().setName("get_i");}
	//	explicit GetInfo(const std::string& name = "getinfo");
	//	ARIS_REGISTER_TYPE(GetInfo);
	//	ARIS_DECLARE_BIG_FOUR(GetInfo);
	//};


}

#endif

