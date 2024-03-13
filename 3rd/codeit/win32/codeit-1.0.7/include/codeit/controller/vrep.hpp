#ifndef CODEIT_CONTROLLER_VREP_H_
#define CODEIT_CONTROLLER_VREP_H_

#include<codeit/core/basictype.hpp>
#include<math.h>
#include<codeit/controller/master_slave.hpp>
#include<codeit/controller/controller_motor.hpp>
#include<codeit/controller/ethercat.hpp>

extern "C" {
#include <codeit/vrep3rd/extApi.h>
}

namespace codeit::controller
{	
	class VrepSlave : virtual public Slave
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		auto smPool()->core::ObjectPool<SyncManager>&;
		auto smPool()const->const core::ObjectPool<SyncManager>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->smPool(); }
		auto vrHandle()->std::string&;
		auto vrHandle()const->std::string& { return const_cast<std::decay_t<decltype(*this)>*>(this)->vrHandle(); }

		auto findPdoEntry(std::uint16_t index, std::uint8_t subindex)const->const PdoEntry* { return const_cast<std::decay_t<decltype(*this)>*>(this)->findPdoEntry(index, subindex); }
		auto findPdoEntry(std::uint16_t index, std::uint8_t subindex)->PdoEntry*;

		template<typename ValueType>
		auto readPdo(std::uint16_t index, std::uint8_t subindex, ValueType& value)const->int { return readPdo(index, subindex, &value, sizeof(ValueType) * 8); }
		auto readPdo(std::uint16_t index, std::uint8_t subindex, void* value, Size bit_size)const->int;
		template<typename ValueType>
		auto writePdo(std::uint16_t index, std::uint8_t subindex, const ValueType& value)->int { return writePdo(index, subindex, &value, sizeof(ValueType) * 8); }
		auto writePdo(std::uint16_t index, std::uint8_t subindex, const void* value, Size bit_size)->int;
		template<typename ValueType>
		auto readSdo(std::uint16_t index, std::uint8_t subindex, ValueType& value)->void { readSdo(index, subindex, &value, sizeof(ValueType)); }
		auto readSdo(std::uint16_t index, std::uint8_t subindex, void* value, Size byte_size)->void;
		template<typename ValueType>
		auto writeSdo(std::uint16_t index, std::uint8_t subindex, const ValueType& value)->void { writeSdo(index, subindex, &value, sizeof(ValueType)); }
		auto writeSdo(std::uint16_t index, std::uint8_t subindex, const void* value, Size byte_size)->void;

		virtual ~VrepSlave();
		explicit VrepSlave(const std::string& name = "vrep_slave", std::uint16_t phy_id = 0, char const* handle="joint0");
		CODEIT_REGISTER_TYPE(VrepSlave);
		VrepSlave(const VrepSlave& other);
		VrepSlave(VrepSlave&& other) = delete;
		VrepSlave& operator=(const VrepSlave& other);
		VrepSlave& operator=(VrepSlave&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;

		friend class VrepMaster;
	};
	class VrepMaster : virtual public Master {
	public:
		
		typedef struct {
			unsigned int link_up : 1; //< ernet link is up.			 
		} MasterLinkState;

		auto slavePool()->core::ChildRefPool<VrepSlave, core::ObjectPool<Slave>>&;
		auto slavePool()const->const core::ChildRefPool<VrepSlave, core::ObjectPool<Slave>>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slavePool(); }
		auto slaveAtAbs(Size id)->VrepSlave& { return dynamic_cast<VrepSlave&>(Master::slaveAtAbs(id)); }
		auto slaveAtAbs(Size id)const->const VrepSlave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtAbs(id); }
		auto slaveAtPhy(Size id)->VrepSlave& { return dynamic_cast<VrepSlave&>(Master::slaveAtPhy(id)); }
		auto slaveAtPhy(Size id)const->const VrepSlave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtPhy(id); }

		auto getLinkState(MasterLinkState* master_state)->void; // only for rt


		auto vrHandle()->int&;
		auto vrHandle()const->const int& { return const_cast<std::decay_t<decltype(*this)>*>(this)->vrHandle(); }

		auto virtual init()->void override;
		auto virtual start()->void override;
		auto virtual stop()->void override;
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		virtual ~VrepMaster();
		VrepMaster(const std::string& name = "vrep_master", const int& port = 20001, const int& nums = 0);
		VrepMaster(const VrepMaster& other) = delete;
		VrepMaster(VrepMaster&& other) = delete;
		VrepMaster& operator=(const VrepMaster& other) = delete;
		VrepMaster& operator=(VrepMaster&& other) = delete;
		CODEIT_REGISTER_TYPE(VrepMaster);

		auto port()->int;
		auto port()const->const int { return const_cast<std::decay_t<decltype(*this)>*>(this)->port(); }
		
		auto ClientID()->simxInt;
		auto ClientID()const->const simxInt { return const_cast<std::decay_t<decltype(*this)>*>(this)->ClientID(); }
		auto SetClientID(simxInt id)->void;
		auto SetClientID(simxInt id)const->const void { return const_cast<std::decay_t<decltype(*this)>*>(this)->SetClientID(id); }

		bool ConnectVrepMaster(const int port);
		bool DisonnectVrepMaster(const int port);
		bool CheckConnected();

		//void startAAA(std::shared_ptr<RtCmdSET>);

	protected:
		auto virtual send()->void override;
		auto virtual recv()->void override;
		auto virtual release()->void override;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
		friend class VrepPdoEntry;
	};
	class VrepMotor :public VrepSlave, public Motor
	{
	public:
		auto isVirtual()->bool;
		auto setVirtual(bool is_virtual = true)->void;

		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		auto virtual controlWord()const->std::uint16_t override;
		auto virtual modeOfOperation()const->std::uint8_t override;
		auto virtual targetPos()const->double override;
		auto virtual targetVel()const->double override;
		auto virtual targetToq()const->double override;
		auto virtual offsetVel()const->double override;
		auto virtual offsetCur()const->double override;
		// require pdo 0x6040 //
		auto virtual setControlWord(std::uint16_t control_word)->void override;
		// require pdo 0x6060 //
		auto virtual setModeOfOperation(std::uint8_t mode)->void override;
		// require pdo 0x607A //
		auto virtual setTargetPos(double pos)->void override;
		// require pdo 0x60FF //
		auto virtual setTargetVel(double vel)->void override;
		// require pdo 0x6071 //
		auto virtual setTargetToq(double toq)->void override;
		// require pdo 0x60B1 //
		auto virtual setOffsetVel(double vel)->void override;
		// require pdo 0x60B2 //
		auto virtual setOffsetToq(double toq)->void override;
		// require pdo 0x6041 //
		auto virtual statusWord()const->std::uint16_t override;
		// require pdo 0x603f or 0x6079 //
		auto virtual errorCode()const->std::int32_t override;
		// require pdo 0x6061 //
		auto virtual modeOfDisplay()const->std::uint8_t override;
		// require pdo 0x6064 //
		auto virtual actualPos()const->double override;
		auto setActualPos(double pos)->void;
		// require pdo 0x606C //
		auto virtual actualVel()const->double override;
		// require pdo 0x6077 //
		auto virtual actualToq()const->double override;
		// require pdo 0x6078 //
		auto virtual actualCur()const->double override;
		// require pdo 0x2064 //
		auto virtual actualAddlPos()const->double override;
		auto virtual velDiff()const->double override;
		auto virtual setVelDiff(double vel)->void override;
		auto virtual setErrorCode(std::int32_t code)->void override { ; }

		// require pdo 0x6040 0x6041 // 
		auto virtual disable()->int override;
		// require pdo 0x6040 0x6041 //
		auto virtual enable()->int override;
		// require pdo 0x6040 0x6041 0x6060 0x6061 //
		auto virtual home()->int override;
		// require pdo 0x6060 0x6061 //
		auto virtual mode(std::uint8_t md)->int override;

		VrepMotor(const std::string& name = "vrep_motor", std::uint16_t phy_id = 0
			, double max_pos = 1.0, double min_pos = -1.0, double max_vel = 1.0, double min_vel = -1.0, double max_acc = 1.0, double min_acc = -1.0
			, double max_pos_following_error = 1.0, double max_vel_following_error = 1.0, double pos_factor = 1.0, double pos_offset = 0.0, double home_pos = 0.0, bool is_virtual = false);
		CODEIT_REGISTER_TYPE(VrepMotor);
		VrepMotor(const VrepMotor& other);
		VrepMotor(VrepMotor&& other) = delete;
		VrepMotor& operator=(const VrepMotor& other);
		VrepMotor& operator=(VrepMotor&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class VrepExternMotor :public VrepSlave, public ExternMotor
	{
	public:
		auto isVirtual()->bool;
		auto setVirtual(bool is_virtual = true)->void;

		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		auto virtual controlWord()const->std::uint16_t override;
		auto virtual modeOfOperation()const->std::uint8_t override;
		auto virtual targetPos()const->double override;
		auto virtual targetVel()const->double override;
		auto virtual targetToq()const->double override;
		auto virtual offsetVel()const->double override;
		auto virtual offsetCur()const->double override;
		// require pdo 0x6040 //
		auto virtual setControlWord(std::uint16_t control_word)->void override;
		// require pdo 0x6060 //
		auto virtual setModeOfOperation(std::uint8_t mode)->void override;
		// require pdo 0x607A //
		auto virtual setTargetPos(double pos)->void override;
		// require pdo 0x60FF //
		auto virtual setTargetVel(double vel)->void override;
		// require pdo 0x6071 //
		auto virtual setTargetToq(double toq)->void override;
		// require pdo 0x60B1 //
		auto virtual setOffsetVel(double vel)->void override;
		// require pdo 0x60B2 //
		auto virtual setOffsetToq(double toq)->void override;
		// require pdo 0x6041 //
		auto virtual statusWord()const->std::uint16_t override;
		// require pdo 0x6061 //
		auto virtual modeOfDisplay()const->std::uint8_t override;
		// require pdo 0x6064 //
		auto virtual actualPos()const->double override;
		auto setActualPos(double pos)->void;
		// require pdo 0x606C //
		auto virtual actualVel()const->double override;
		// require pdo 0x6077 //
		auto virtual actualToq()const->double override;
		// require pdo 0x6078 //
		auto virtual actualCur()const->double override;

		// require pdo 0x6040 0x6041 // 
		auto virtual disable()->int override;
		// require pdo 0x6040 0x6041 //
		auto virtual enable()->int override;
		// require pdo 0x6040 0x6041 0x6060 0x6061 //
		auto virtual home()->int override;
		// require pdo 0x6060 0x6061 //
		auto virtual mode(std::uint8_t md)->int override;

		VrepExternMotor(const std::string& name = "vrep_motor", std::uint16_t phy_id = 0
			, double max_pos = 1.0, double min_pos = -1.0, double max_vel = 1.0, double min_vel = -1.0, double max_acc = 1.0, double min_acc = -1.0
			, double max_pos_following_error = 1.0, double max_vel_following_error = 1.0, double pos_factor = 1.0, double pos_offset = 0.0, double home_pos = 0.0, bool is_virtual = false);
		CODEIT_REGISTER_TYPE(VrepExternMotor);
		VrepExternMotor(const VrepExternMotor& other);
		VrepExternMotor(VrepExternMotor&& other) = delete;
		VrepExternMotor& operator=(const VrepExternMotor& other);
		VrepExternMotor& operator=(VrepExternMotor&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class VrepController :public VrepMaster, public Controller
	{
	public:
		using MotionPool = core::SubRefPool<VrepMotor, core::ChildRefPool<VrepSlave, core::ObjectPool<Slave>>>;
		auto motionPool()->MotionPool&;
		auto motionPool()const->const MotionPool& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionPool(); }
		auto motionAtAbs(Size id)->VrepMotor& { return dynamic_cast<VrepMotor&>(Controller::motionAtAbs(id)); }
		auto motionAtAbs(Size id)const->const VrepMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionAtAbs(id); }
		auto motionAtPhy(Size id)->VrepMotor& { return dynamic_cast<VrepMotor&>(Controller::motionAtPhy(id)); }
		auto motionAtPhy(Size id)const->const VrepMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionAtPhy(id); }
		auto motionAtSla(Size id)->VrepMotor& { return dynamic_cast<VrepMotor&>(Controller::motionAtSla(id)); }
		auto motionAtSla(Size id)const->const VrepMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionAtSla(id); }

		using ExternMotionPool = core::SubRefPool<VrepExternMotor, core::ChildRefPool<VrepSlave, core::ObjectPool<Slave>>>;
		auto externMotionPool()->ExternMotionPool&;
		auto externMotionPool()const->const MotionPool& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionPool(); }
		auto externMotionAtAbs(Size id)->VrepExternMotor& { return dynamic_cast<VrepExternMotor&>(Controller::motionAtAbs(id)); }
		auto externMotionAtAbs(Size id)const->const VrepExternMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->externMotionAtAbs(id); }
		auto externMotionAtPhy(Size id)->VrepExternMotor& { return dynamic_cast<VrepExternMotor&>(Controller::motionAtPhy(id)); }
		auto externMotionAtPhy(Size id)const->const VrepExternMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->externMotionAtPhy(id); }
		auto externMotionAtSla(Size id)->VrepExternMotor& { return dynamic_cast<VrepExternMotor&>(Controller::motionAtSla(id)); }
		auto externMotionAtSla(Size id)const->const VrepExternMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->externMotionAtSla(id); }


		auto slavePool()->core::ChildRefPool<VrepSlave, core::ObjectPool<Slave>>& { return VrepMaster::slavePool(); }
		auto slavePool()const->const core::ChildRefPool<VrepSlave, core::ObjectPool<Slave>>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slavePool(); }
		auto slaveAtAbs(Size id)->VrepSlave& { return VrepMaster::slaveAtAbs(id); }
		auto slaveAtAbs(Size id)const->const VrepSlave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtAbs(id); }
		auto slaveAtPhy(Size id)->VrepSlave& { return VrepMaster::slaveAtPhy(id); }
		auto slaveAtPhy(Size id)const->const VrepSlave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtPhy(id); }

		auto virtual start()->void override { VrepMaster::start(); }
		auto virtual stop()->void override { VrepMaster::stop(); }
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override { VrepMaster::saveXml(xml_ele); }
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override { VrepMaster::loadXml(xml_ele); }

		virtual ~VrepController();
		VrepController(const std::string& name = "vrep_controller");
		VrepController(const VrepController& other) = delete;
		VrepController(VrepController&& other) = delete;
		VrepController& operator=(const VrepController& other) = delete;
		VrepController& operator=(VrepController&& other) = delete;
		CODEIT_REGISTER_TYPE(VrepController);

	protected:
		auto virtual init()->void override;
		auto virtual send()->void override { VrepMaster::send(); }
		auto virtual recv()->void override { VrepMaster::recv(); }
		auto virtual release()->void override { VrepMaster::release(); }

		struct Imp;
		core::ImpPtr<Imp> imp_;
	};

}
#endif