#ifndef CODEIT_IO_MODEL_H_
#define CODEIT_IO_MODEL_H_

#include <codeit/core/object.hpp>

namespace codeit::model
{
	class IOModel;

	class IOElement :public core::Object
	{
	public:
		enum IO_TYPE : std::uint64_t
		{
			DI_DEVICE = 0x01ULL << 0,
			DO_DEVICE = 0x01ULL << 1,
			AI_DEVICE = 0x01ULL << 2,
			AO_DEVICE = 0x01ULL << 3,
		};
		auto ioId()const->Size { return io_id_; }
		auto ioModel()noexcept->IOModel& { return *ancestor<IOModel>(); }
		auto ioModel()const noexcept->const IOModel& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioModel(); }
		auto active() const noexcept->bool { return active_; }
		auto virtual ioType()->IO_TYPE { return io_type; }
		auto virtual setIoType(IO_TYPE type_)->void { io_type = type_; }

		~IOElement()=default;
		explicit IOElement(const std::string& name = "ioelement") :Object(name) {}
		CODEIT_REGISTER_TYPE(IOElement);
		CODEIT_DEFINE_BIG_FOUR(IOElement);
		
	private:
		Size io_id_;
		IO_TYPE io_type;
		bool active_;
	};

	class DI : public IOElement
	{
	public:
		auto virtual actualDi()->bool { return recv_di.load(); }
		auto virtual setDi(bool recv_di_)->void { recv_di.store(recv_di_);}
		
		~DI() = default;
		explicit DI(const std::string& name = "di") :IOElement(name) { setIoType(IO_TYPE::DI_DEVICE); }
		CODEIT_REGISTER_TYPE(DI);
		CODEIT_DEFINE_BIG_FOUR(DI);
	private:
		atomic_bool recv_di;
		
	};
	class DO : public IOElement
	{
	public:
		auto virtual actualDo()->bool { return send_do.load(); }
		auto virtual setDo(bool send_do_)->void { send_do.store(send_do_); }
		
		auto virtual pulseActive()->bool { return pulse_active; }
		auto virtual setPulseActive(bool pulse_active_)->void { pulse_active = pulse_active_; }

		auto virtual highCycles()->std::int32_t { return high_cycles; }
		auto virtual setHighCycles(std::int32_t high_cycles_)->void { high_cycles = high_cycles_; }
		auto virtual lowCycles()->std::int32_t { return low_cycles; }
		auto virtual setLowCycles(std::int32_t low_cycles_)->void { low_cycles = low_cycles_; }

		auto virtual saveXml(core::XmlElement& xml_ele) const->void override
		{
			IOElement::saveXml(xml_ele);
			xml_ele.SetAttribute("pulse_active", pulse_active);
			if (pulse_active) {
				xml_ele.SetAttribute("high_cycles", high_cycles);
				xml_ele.SetAttribute("low_cycles", low_cycles);
			}

		}
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override
		{
			IOElement::loadXml(xml_ele);
			pulse_active = Object::attributeBool(xml_ele, "pulse_active", false);
			high_cycles = Object::attributeInt32(xml_ele, "high_cycles", 1);
			low_cycles = Object::attributeInt32(xml_ele, "low_cycles", 1);

		}


		~DO() = default;
		explicit DO(const std::string& name = "do",const bool active = false,\
		const std::int32_t high_cycles_=1, const std::int32_t low_cycles_ = 1) :IOElement(name)
		{ 
			setIoType(IO_TYPE::DO_DEVICE); 
			pulse_active = active;
			high_cycles = high_cycles_;
			low_cycles = low_cycles_;
		}
		CODEIT_REGISTER_TYPE(DO);
		CODEIT_DEFINE_BIG_FOUR(DO);
	private:
		atomic_bool send_do;
		
		bool pulse_active{ false };
		std::int32_t high_cycles{ 1 };
		std::int32_t low_cycles{ 1 };
	};
	class AI : public IOElement
	{
	public:
		auto virtual actualAi()->double { return recv_ai.load(); }
		auto virtual setAi(double recv_ai_)->void { recv_ai.store(recv_ai_); }
		~AI() = default;
		explicit AI(const std::string& name = "ai") :IOElement(name) { setIoType(IO_TYPE::AI_DEVICE); }
		CODEIT_REGISTER_TYPE(AI);
		CODEIT_DEFINE_BIG_FOUR(AI);
	private:
		atomic<double> recv_ai;
		
	};
	class AO : public IOElement
	{
	public:
		auto virtual actualAo()->double { return recv_ao.load(); }
		auto virtual setAo(double recv_ao_)->void { recv_ao.store(recv_ao_); }
		~AO() = default;
		explicit AO(const std::string& name = "ao") :IOElement(name) { setIoType(IO_TYPE::AO_DEVICE); }
		CODEIT_REGISTER_TYPE(AO);
		CODEIT_DEFINE_BIG_FOUR(AO);
	private:
		atomic<double> recv_ao;
		
	};

	class SubSysElement :public core::Object
	{
	public:
		
		auto subsysId()const->Size { return subsys_id_; }
		auto ioModel()noexcept->IOModel& { return *ancestor<IOModel>(); }
		auto ioModel()const noexcept->const IOModel& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioModel(); }
		auto setActive(bool active) noexcept->void { active_.store(active); }
		auto active() const noexcept->bool { return active_.load(); }
		auto virtual actualCmd()->std::string
		{
			std::unique_lock<std::mutex> lck(cmd_mutex_);
			return cmd;

		}
		auto virtual setCmd(std::string cmd_)->void
		{
			std::unique_lock<std::mutex> lck(cmd_mutex_);
			cmd = cmd_;
		}

		~SubSysElement() = default;
		explicit SubSysElement(const std::string& name = "subsys_element") :Object(name) {}
		CODEIT_REGISTER_TYPE(SubSysElement);
		CODEIT_DEFINE_BIG_FOUR(SubSysElement);

	private:
		Size subsys_id_;
		std::atomic_bool active_;
		std::string cmd;
		std::mutex cmd_mutex_;
	};
	class IOModel :public core::Object
	{
	public:
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto virtual saveXml(core::XmlElement& xml_ele)const->void override;
		auto ioPool()->core::ObjectPool<IOElement, Object>&;
		auto ioPool()const->const core::ObjectPool<IOElement, Object>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioPool(); }
		auto subsysPool()->core::ObjectPool<SubSysElement, Object>&;
		auto subsysPool()const->const core::ObjectPool<SubSysElement, Object>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->subsysPool(); }

		virtual ~IOModel();
		explicit IOModel(const std::string& name = "iomodel");
		CODEIT_REGISTER_TYPE(IOModel);
		CODEIT_DECLARE_BIG_FOUR(IOModel);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;

	};
	


}

#endif
