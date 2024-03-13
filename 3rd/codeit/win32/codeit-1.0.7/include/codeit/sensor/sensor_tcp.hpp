#ifndef codeit_SENSOR_TCP_H_
#define codeit_SENSOR_TCP_H_
#include <codeit/sensor/sensor_base.hpp>

namespace codeit::sensor
{
	class SensorTCP :public Sensor
	{
	public:
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual start()->void override;
		auto virtual stop()->void override;
	
		SensorTCP(const std::string& name = "sensor_tcp", const std::string& remote_ip = "", const std::string& port = "5866", core::Socket::TYPE type = core::Socket::WEB, const Size& length = 0, const bool& is_server=true);
		CODEIT_REGISTER_TYPE(SensorTCP);
		CODEIT_DEFINE_BIG_FOUR(SensorTCP);

	protected:
		auto virtual init()->void override;
		auto virtual release()->void override { Sensor::release(); }
		auto virtual updateData(core::Msg& data)->void override;

	private:
		core::Socket* sock_;
		struct Imp;
		std::unique_ptr<Imp> imp_;
		
	};
}

#endif
