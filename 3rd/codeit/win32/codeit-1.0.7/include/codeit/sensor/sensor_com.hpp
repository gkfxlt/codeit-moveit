#ifndef codeit_SENSOR_COM_H_
#define codeit_SENSOR_COM_H_
#include <codeit/sensor/sensor_base.hpp>

namespace codeit::sensor
{
	class SensorCOM :public Sensor
	{
	public:
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual start()->void override;
		auto virtual stop()->void override;

		SensorCOM(const std::string& name = "sensor_com", const core::SerialPort::ComOptions& options = core::SerialPort::defaultComOptions, const Size& length = 0);
		CODEIT_REGISTER_TYPE(SensorCOM);
		CODEIT_DEFINE_BIG_FOUR(SensorCOM);

	protected:
		auto virtual init()->void override;
		auto virtual release()->void override { Sensor::release(); }
		auto virtual updateData(core::Msg& data)->void override;

	private:
		core::SerialPort* com_;
		Size length_, interval_;
	};


}

#endif
