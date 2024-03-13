#ifndef CODEIT_CONTROLLER_CAN_H_
#define CODEIT_CONTROLLER_CAN_H_

#include <codeit/controller/ethercat.hpp>
#include <codeit/can3rd/usb2can.hpp>

namespace codeit::controller
{
#define NMT	   0x0
#define SYNC       0x1
#define TIME_STAMP 0x2
#define PDO1tx     0x3
#define PDO1rx     0x4
#define PDO2tx     0x5
#define PDO2rx     0x6
#define PDO3tx     0x7
#define PDO3rx     0x8
#define PDO4tx     0x9
#define PDO4rx     0xA
#define SDOtx      0xB
#define SDOrx      0xC
#define NODE_GUARD 0xE
#define LSS 	   0xF
	class NrtCanMaster;
	class CanSlave : virtual public Slave
	{
	public:
		enum CAN_SLAVE_STATE : UNS8
		{
			Initialisation = 0x00,
			Disconnected = 0x01,
			Connecting = 0x02,
			Preparing = 0x02,
			Stopped = 0x04,
			Operational = 0x05,
			Pre_operational = 0x7F,
			Unknown_state = 0x0F
		};
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		auto vendorID()const->std::uint32_t;
		auto setVendorID(std::uint32_t vendor_id)->void;
		auto productCode()const->std::uint32_t;
		auto setProductCode(std::uint32_t product_code)->void;
		auto revisionNum()const->std::uint32_t;
		auto setRevisionNum(std::uint32_t revision_num)->void;
		auto virtual slaveState()const->UNS8;
		auto setSlaveState(UNS8 slave_state)->void;

		auto heartBeatLast()const->clock_t;
		auto setHeartBeatLast(clock_t time)->void;

		/*auto findPdoEntry(std::uint16_t index, std::uint8_t subindex)const->const PdoEntry* { return const_cast<std::decay_t<decltype(*this)>*>(this)->findPdoEntry(index, subindex); }
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
*/	
		virtual ~CanSlave();
		explicit CanSlave(const std::string& name = "can_slave", std::uint16_t phy_id = 0);
		CODEIT_REGISTER_TYPE(CanSlave);
		CanSlave(const CanSlave& other)=delete;
		CanSlave(CanSlave&& other) = delete;
		CanSlave& operator=(const CanSlave& other)=delete;
		CanSlave& operator=(CanSlave&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;

		friend class CanMaster;
	};
	class CanMaster : virtual public Master, public CanDriver
	{
	public:
		auto buildRPDO(UNS8 nodeId, UNS8 pdoNum, UNS16 index0, UNS8 sub_index0, UNS8 size0, UNS16 index1, UNS8 sub_index1, UNS8 size1)->void;
		auto buildTPDO(UNS8 nodeId, UNS8 pdoNum, UNS16 index0, UNS8 sub_index0, UNS8 size0, UNS16 index1, UNS8 sub_index1, UNS8 size1)->void;

		auto slavePool()->core::ChildRefPool<CanSlave, core::ObjectPool<Slave>>&;
		auto slavePool()const->const core::ChildRefPool<CanSlave, core::ObjectPool<Slave>>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slavePool(); }
		auto slaveAtAbs(Size id)->CanSlave& { return dynamic_cast<CanSlave&>(Master::slaveAtAbs(id)); }
		auto slaveAtAbs(Size id)const->const CanSlave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtAbs(id); }
		auto slaveAtPhy(Size id)->CanSlave& { return dynamic_cast<CanSlave&>(Master::slaveAtPhy(id)); }
		auto slaveAtPhy(Size id)const->const CanSlave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtPhy(id); }

		auto ecHandle()->std::any&;
		auto ecHandle()const->const std::any& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ecHandle(); }
		auto scan()->void;

		void proceedSDO(CanMessage& m);
		void proceedPDO(CanMessage& m);
		void proceedNODE_GUARD(CanMessage& m);
		void proceedNMTstateChange(CanMessage& m);
		auto virtual init()->void override;
		auto virtual start()->void override;
		//auto virtual stop()->void override;
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		bool masterReady();
		void setMasterReady(bool);
		bool coutCan();
		void setCoutCan(bool);
		auto writeSdo(Size id, const UNS8 data[8])->void;

		auto heartBeatNow()const->clock_t;
		auto setHeartBeatNow(clock_t time)->void;
		virtual ~CanMaster();
		CanMaster(const std::string& name = "can_master", const CanDriver::CanBoard& board = CanDriver::defaultCanBoard, const bool cout_can=true);
		CanMaster(const CanMaster& other) = delete;
		CanMaster(CanMaster&& other) = delete;
		CanMaster& operator=(const CanMaster& other) = delete;
		CanMaster& operator=(CanMaster&& other) = delete;
		CODEIT_REGISTER_TYPE(CanMaster);

	protected:
		auto virtual send()->void override;
		auto virtual recv()->void override;
		auto virtual release()->void override;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;

	};

	class CanMotor :public CanSlave, public Motor
	{
	public:
		auto isVirtual()->bool;
		auto setVirtual(bool is_virtual = true)->void;
		auto virtual slaveState()const->UNS8;

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
		auto virtual setErrorCode(std::int32_t code)->void override;
		// require pdo 0x6040 0x6041 // 
		auto virtual disable()->int override;
		// require pdo 0x6040 0x6041 //
		auto virtual enable()->int override;
		// require pdo 0x6040 0x6041 0x6060 0x6061 //
		auto virtual home()->int override;
		// require pdo 0x6060 0x6061 //
		auto virtual mode(std::uint8_t md)->int override;

		CanMotor(const std::string& name = "can_motion", std::uint16_t phy_id = 0
			, std::uint32_t vendor_id = 0x00000000, std::uint32_t product_code = 0x00000000, std::uint32_t revision_num = 0x00000000
			, double max_pos = 1.0, double min_pos = -1.0, double max_vel = 1.0, double min_vel = -1.0, double max_acc = 1.0, double min_acc = -1.0
			, double max_pos_following_error = 1.0, double max_vel_following_error = 1.0, double pos_factor = 1.0, double pos_offset = 0.0, double home_pos = 0.0, bool is_virtual = false);
		CODEIT_REGISTER_TYPE(CanMotor);
		CanMotor(const CanMotor& other)=delete;
		CanMotor(CanMotor&& other) = delete;
		CanMotor& operator=(const CanMotor& other)=delete;
		CanMotor& operator=(CanMotor&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
		friend class CanMaster;
		friend class NrtCanMaster;
	};
	
	class CanController :public CanMaster, public Controller
	{
	public:
		using MotionPool = core::SubRefPool<CanMotor, core::ChildRefPool<CanSlave, core::ObjectPool<Slave>>>;
		auto motionPool()->MotionPool&;
		auto motionPool()const->const MotionPool& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionPool(); }
		auto motionAtAbs(Size id)->CanMotor& { return dynamic_cast<CanMotor&>(Controller::motionAtAbs(id)); }
		auto motionAtAbs(Size id)const->const CanMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionAtAbs(id); }
		auto motionAtPhy(Size id)->CanMotor& { return dynamic_cast<CanMotor&>(Controller::motionAtPhy(id)); }
		auto motionAtPhy(Size id)const->const CanMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionAtPhy(id); }
		auto motionAtSla(Size id)->CanMotor& { return dynamic_cast<CanMotor&>(Controller::motionAtSla(id)); }
		auto motionAtSla(Size id)const->const CanMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionAtSla(id); }

		/*using ExternMotionPool = core::SubRefPool<EthercatExternMotor, core::ChildRefPool<EthercatSlave, core::ObjectPool<Slave>>>;
		auto externMotionPool()->ExternMotionPool&;
		auto externMotionPool()const->const ExternMotionPool& { return const_cast<std::decay_t<decltype(*this)>*>(this)->externMotionPool(); }
		auto externMotionAtAbs(Size id)->EthercatExternMotor& { return dynamic_cast<EthercatExternMotor&>(Controller::externMotionAtAbs(id)); }
		auto externMotionAtAbs(Size id)const->const EthercatExternMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->externMotionAtAbs(id); }
		auto externMotionAtPhy(Size id)->EthercatExternMotor& { return dynamic_cast<EthercatExternMotor&>(Controller::externMotionAtPhy(id)); }
		auto externMotionAtPhy(Size id)const->const EthercatExternMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->externMotionAtPhy(id); }
		auto externMotionAtSla(Size id)->EthercatExternMotor& { return dynamic_cast<EthercatExternMotor&>(Controller::externMotionAtSla(id)); }
		auto externMotionAtSla(Size id)const->const EthercatExternMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->externMotionAtSla(id); }
*/

		auto slavePool()->core::ChildRefPool<CanSlave, core::ObjectPool<Slave>>& { return CanMaster::slavePool(); }
		auto slavePool()const->const core::ChildRefPool<CanSlave, core::ObjectPool<Slave>>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slavePool(); }
		auto slaveAtAbs(Size id)->CanSlave& { return CanMaster::slaveAtAbs(id); }
		auto slaveAtAbs(Size id)const->const CanSlave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtAbs(id); }
		auto slaveAtPhy(Size id)->CanSlave& { return CanMaster::slaveAtPhy(id); }
		auto slaveAtPhy(Size id)const->const CanSlave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtPhy(id); }

		/*using IOPool = core::SubRefPool<EthercatIO, core::ChildRefPool<CanSlave, core::ObjectPool<Slave>>>;
		auto ioPool()->IOPool&;
		auto ioPool()const->const IOPool& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioPool(); }
		auto ioAtAbs(Size id)->EthercatIO& { return dynamic_cast<EthercatIO&>(Controller::ioAtAbs(id)); }
		auto ioAtAbs(Size id)const->const EthercatIO& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioAtAbs(id); }
		auto ioAtPhy(Size id)->EthercatIO& { return dynamic_cast<EthercatIO&>(Controller::ioAtPhy(id)); }
		auto ioAtPhy(Size id)const->const EthercatIO& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioAtPhy(id); }
		auto ioAtSla(Size id)->EthercatIO& { return dynamic_cast<EthercatIO&>(Controller::ioAtSla(id)); }
		auto ioAtSla(Size id)const->const EthercatIO& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioAtSla(id); }
*/ 
		auto virtual start()->void override { CanMaster::start(); }
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override { CanMaster::saveXml(xml_ele); }
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override { CanMaster::loadXml(xml_ele); }
		virtual ~CanController();
		CanController(const std::string& name = "can_controller", const CanDriver::CanBoard& board = CanDriver::defaultCanBoard, const bool cout_can = true);
		CanController(const CanController& other) = delete;
		CanController(CanController&& other) = delete;
		CanController& operator=(const CanController& other) = delete;
		CanController& operator=(CanController&& other) = delete;
		CODEIT_REGISTER_TYPE(CanController);

	protected:
		auto virtual init()->void override;
		auto virtual send()->void override { CanMaster::send(); }
		auto virtual recv()->void override { CanMaster::recv(); }
		auto virtual release()->void override { CanMaster::release(); }

		struct Imp;
		core::ImpPtr<Imp> imp_;
	};

	
	class NrtCanMaster : virtual public NrtMaster, public CanDriver
	{
	public:
		auto buildRPDO(UNS8 nodeId, UNS8 pdoNum, UNS16 index0, UNS8 sub_index0, UNS8 size0, UNS16 index1, UNS8 sub_index1, UNS8 size1)->void;
		auto buildTPDO(UNS8 nodeId, UNS8 pdoNum, UNS16 index0, UNS8 sub_index0, UNS8 size0, UNS16 index1, UNS8 sub_index1, UNS8 size1)->void;

		auto slavePool()->core::ChildRefPool<CanSlave, core::ObjectPool<Slave>>&;
		auto slavePool()const->const core::ChildRefPool<CanSlave, core::ObjectPool<Slave>>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slavePool(); }
		auto slaveAtAbs(Size id)->CanSlave& { return dynamic_cast<CanSlave&>(NrtMaster::slaveAtAbs(id)); }
		auto slaveAtAbs(Size id)const->const CanSlave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtAbs(id); }
		auto slaveAtPhy(Size id)->CanSlave& { return dynamic_cast<CanSlave&>(NrtMaster::slaveAtPhy(id)); }
		auto slaveAtPhy(Size id)const->const CanSlave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtPhy(id); }

		auto ecHandle()->std::any&;
		auto ecHandle()const->const std::any& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ecHandle(); }
		auto scan()->void;

		void proceedSDO(CanMessage& m);
		void proceedPDO(CanMessage& m);
		void proceedNODE_GUARD(CanMessage& m);
		void proceedNMTstateChange(CanMessage& m);
		auto virtual init()->void override;
		auto virtual start()->void override;
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		bool masterReady();
		void setMasterReady(bool);
		bool coutCan();
		void setCoutCan(bool);
		auto writeSdo(Size id, const UNS8 data[8])->void;

		auto heartBeatNow()const->clock_t;
		auto setHeartBeatNow(clock_t time)->void;
		virtual ~NrtCanMaster();
		NrtCanMaster(const std::string& name = "can_master", const CanDriver::CanBoard& board = CanDriver::defaultCanBoard, const bool cout_can = true);
		NrtCanMaster(const NrtCanMaster& other) = delete;
		NrtCanMaster(NrtCanMaster&& other) = delete;
		NrtCanMaster& operator=(const NrtCanMaster& other) = delete;
		NrtCanMaster& operator=(NrtCanMaster&& other) = delete;
		CODEIT_REGISTER_TYPE(NrtCanMaster);

	protected:
		auto virtual send()->void override;
		auto virtual recv()->void override;
		auto virtual release()->void override;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;

	};


	class NrtCanController :public NrtCanMaster, public NrtController
	{
	public:
		using MotionPool = core::SubRefPool<CanMotor, core::ChildRefPool<CanSlave, core::ObjectPool<Slave>>>;
		auto motionPool()->MotionPool&;
		auto motionPool()const->const MotionPool& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionPool(); }
		auto motionAtAbs(Size id)->CanMotor& { return dynamic_cast<CanMotor&>(NrtController::motionAtAbs(id)); }
		auto motionAtAbs(Size id)const->const CanMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionAtAbs(id); }
		auto motionAtPhy(Size id)->CanMotor& { return dynamic_cast<CanMotor&>(NrtController::motionAtPhy(id)); }
		auto motionAtPhy(Size id)const->const CanMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionAtPhy(id); }
		auto motionAtSla(Size id)->CanMotor& { return dynamic_cast<CanMotor&>(NrtController::motionAtSla(id)); }
		auto motionAtSla(Size id)const->const CanMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionAtSla(id); }

		auto slavePool()->core::ChildRefPool<CanSlave, core::ObjectPool<Slave>>& { return NrtCanMaster::slavePool(); }
		auto slavePool()const->const core::ChildRefPool<CanSlave, core::ObjectPool<Slave>>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slavePool(); }
		auto slaveAtAbs(Size id)->CanSlave& { return NrtCanMaster::slaveAtAbs(id); }
		auto slaveAtAbs(Size id)const->const CanSlave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtAbs(id); }
		auto slaveAtPhy(Size id)->CanSlave& { return NrtCanMaster::slaveAtPhy(id); }
		auto slaveAtPhy(Size id)const->const CanSlave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtPhy(id); }

		auto virtual start()->void override { NrtCanMaster::start(); }
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override { NrtCanMaster::saveXml(xml_ele); }
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override { NrtCanMaster::loadXml(xml_ele); }
		virtual ~NrtCanController();
		NrtCanController(const std::string& name = "nrt_can_controller", const CanDriver::CanBoard& board = CanDriver::defaultCanBoard, const bool cout_can = true);
		NrtCanController(const NrtCanController& other) = delete;
		NrtCanController(NrtCanController&& other) = delete;
		NrtCanController& operator=(const NrtCanController& other) = delete;
		NrtCanController& operator=(NrtCanController&& other) = delete;
		CODEIT_REGISTER_TYPE(NrtCanController);

	protected:
		auto virtual init()->void override;
		auto virtual send()->void override { NrtCanMaster::send(); }
		auto virtual recv()->void override { NrtCanMaster::recv(); }
		auto virtual release()->void override { NrtCanMaster::release(); }

		struct Imp;
		core::ImpPtr<Imp> imp_;
	};

}

#endif