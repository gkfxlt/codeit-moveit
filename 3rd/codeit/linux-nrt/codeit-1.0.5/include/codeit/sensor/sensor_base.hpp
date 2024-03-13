#ifndef codeit_SENSOR_BASE_H_
#define codeit_SENSOR_BASE_H_

#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>
#include <memory>

#include <codeit/core/core.hpp>
#include <codeit/sensor/sensor_data.hpp>

namespace codeit::sensor
{
	class Sensor;
	
	class SensorDataProtector
	{
	public:
		auto get() const->const core::Msg* { return data_; }
		auto data() const->const core::Msg& { return *data_; }
		auto operator->()const -> const core::Msg* { return data_; }
		auto operator*()const -> const core::Msg& { return std::ref(*data_); }
		auto operator=(SensorDataProtector && other)->SensorDataProtector & { std::swap(*this, other); return *this; }

		~SensorDataProtector() = default;
		SensorDataProtector() : sensor_(nullptr), data_(nullptr) {}
		SensorDataProtector(SensorDataProtector && other) = default;

	private:
		explicit SensorDataProtector(Sensor* sensor);
		
		SensorDataProtector(const SensorDataProtector&) = delete;
		SensorDataProtector & operator=(const SensorDataProtector&) = delete;

		Sensor *sensor_;
		const core::Msg* data_;
		std::unique_lock<std::recursive_mutex> lock_;

		friend class Sensor;
	};
	class Sensor :public core::Object
	{
	public:
		//static auto Type()->const std::string & { static const std::string type("Sensor"); return std::ref(type); }
		//auto virtual type() const->const std::string& override { return Type(); }
		auto virtual start()->void;
		auto virtual stop()->void;
		auto length()->Size;
		auto setLength(Size length_)->void;
		auto dataProtector()->SensorDataProtector;
		template<class SensorData> void display(SensorData& data)
		{
			auto msg = dataProtector().get();
			memcpy(&data, msg->data(), length());
		}

		virtual ~Sensor();
		Sensor(const std::string &name = "sensor", const Size& length = 0);
		CODEIT_REGISTER_TYPE(Sensor);

	protected:
		auto virtual init()->void {}
		auto virtual release()->void {}
		auto virtual updateData(core::Msg& data)->void {}

	private:
		auto operator=(const Sensor &)->Sensor& = default;
		auto operator=(Sensor &&)->Sensor& = default;
		Sensor(const Sensor &) = default;
		Sensor(Sensor &&) = default;

		struct Imp;
		core::ImpPtr<Imp> imp_;

		friend class SensorDataProtector;
	};

}


#endif
