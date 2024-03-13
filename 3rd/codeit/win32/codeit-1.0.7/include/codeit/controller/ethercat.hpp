#ifndef CODEIT_CONTROLLER_ETHERCAT_H_
#define CODEIT_CONTROLLER_ETHERCAT_H_

#include <filesystem>

#include <codeit/controller/master_slave.hpp>
#include<codeit/controller/controller_motor.hpp>
namespace codeit::controller
{

	class PdoEntry :public core::Object
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		auto ecHandle()->std::any&;
		auto ecHandle()const->const std::any& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ecHandle(); }
		auto index()const->std::uint16_t;
		auto subindex()const->std::uint8_t;
		auto bitSize()const->Size;

		virtual ~PdoEntry();
		explicit PdoEntry(const std::string& name = "entry", std::uint16_t index = 0x0000, std::uint8_t subindex = 0x00, Size bit_size = 8);
		CODEIT_REGISTER_TYPE(PdoEntry);
		CODEIT_DECLARE_BIG_FOUR(PdoEntry);

	public:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class Pdo :public core::ObjectPool<PdoEntry>
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto ecHandle()->std::any&;
		auto ecHandle()const->const std::any& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ecHandle(); }
		auto index()const->std::uint16_t;

		virtual ~Pdo();
		explicit Pdo(const std::string& name = "pdo", std::uint16_t index = 0x0000);
		CODEIT_REGISTER_TYPE(Pdo);
		CODEIT_DECLARE_BIG_FOUR(Pdo);

	public:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class SyncManager :public core::ObjectPool<Pdo>
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto tx()const->bool;
		auto rx()const->bool;

		virtual ~SyncManager();
		explicit SyncManager(const std::string& name = "sm", bool is_tx = true);
		CODEIT_REGISTER_TYPE(SyncManager);
		CODEIT_DECLARE_BIG_FOUR(SyncManager);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};

	class EthercatSlave : virtual public Slave
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		auto smPool()->core::ObjectPool<SyncManager>&;
		auto smPool()const->const core::ObjectPool<SyncManager>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->smPool(); }
		auto ecHandle()->std::any&;
		auto ecHandle()const->const std::any& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ecHandle(); }

		auto vendorID()const->std::uint32_t;
		auto setVendorID(std::uint32_t vendor_id)->void;
		auto productCode()const->std::uint32_t;
		auto setProductCode(std::uint32_t product_code)->void;
		auto revisionNum()const->std::uint32_t;
		auto setRevisionNum(std::uint32_t revision_num)->void;
		auto dcAssignActivate()const->std::uint32_t;
		auto setDcAssignActivate(std::uint32_t dc_assign_activate)->void;
		auto sync0ShiftNs()const->std::int32_t;
		auto setSync0ShiftNs(std::int32_t sync0_shift_ns)->void;
		auto scanInfoForCurrentSlave()->void;
		auto scanPdoForCurrentSlave()->void;
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

		virtual ~EthercatSlave();
		explicit EthercatSlave(const std::string& name = "ethercat_slave", std::uint16_t phy_id = 0, std::uint32_t vendor_id = 0, std::uint32_t product_code = 0, std::uint32_t revision_num = 0, std::uint32_t dc_assign_activate = 0x300, std::int32_t sync0_shift_ns = 650'000);
		CODEIT_REGISTER_TYPE(EthercatSlave);
		EthercatSlave(const EthercatSlave& other);
		EthercatSlave(EthercatSlave&& other) = delete;
		EthercatSlave& operator=(const EthercatSlave& other);
		EthercatSlave& operator=(EthercatSlave&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;

		friend class EthercatMaster;
	};
	class EthercatMaster : virtual public Master
	{
	public:
		typedef struct {
			unsigned int online : 1; /**< The slave is online. */
			unsigned int operational : 1; /**< The slave was brought into \a OP state
										  using the specified configuration. */
			unsigned int al_state : 4; /**< The application-layer state of the slave.
									   - 1: \a INIT
									   - 2: \a PREOP
									   - 4: \a SAFEOP
									   - 8: \a OP

									   Note that each state is coded in a different
									   bit! */
		} SlaveLinkState;
		typedef struct {
			unsigned int slaves_responding; /**< Sum of responding slaves on the given
											link. */
			unsigned int al_states : 4; /**< Application-layer states of the slaves on
										the given link.  The states are coded in the
										lower 4 bits.  If a bit is set, it means
										that at least one slave in the bus is in the
										corresponding state:
										- Bit 0: \a INIT
										- Bit 1: \a PREOP
										- Bit 2: \a SAFEOP
										- Bit 3: \a OP */
			unsigned int link_up : 1; /**< \a true, if the given Ethernet link is up.
									  */
		} MasterLinkState;


		auto slavePool()->core::ChildRefPool<EthercatSlave, core::ObjectPool<Slave>>&;
		auto slavePool()const->const core::ChildRefPool<EthercatSlave, core::ObjectPool<Slave>>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slavePool(); }
		auto slaveAtAbs(Size id)->EthercatSlave& { return dynamic_cast<EthercatSlave&>(Master::slaveAtAbs(id)); }
		auto slaveAtAbs(Size id)const->const EthercatSlave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtAbs(id); }
		auto slaveAtPhy(Size id)->EthercatSlave& { return dynamic_cast<EthercatSlave&>(Master::slaveAtPhy(id)); }
		auto slaveAtPhy(Size id)const->const EthercatSlave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtPhy(id); }

		auto getLinkState(MasterLinkState* master_state, SlaveLinkState* slave_state)->void; // only for rt

		auto ecHandle()->std::any&;
		auto ecHandle()const->const std::any& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ecHandle(); }
		auto scan()->void;
		auto scanInfoForCurrentSlaves()->void;
		auto scanPdoForCurrentSlaves()->void;

		auto setEsiDirs(std::vector<std::filesystem::path> esi_dirs)->void;
		auto updateDeviceList()->void;
		auto getDeviceList()->std::string;
		auto getPdoList(int vendor_id, int product_code, int revision_num)->std::string;

		auto virtual init()->void override;
		auto virtual start()->void override;
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		virtual ~EthercatMaster();
		EthercatMaster(const std::string& name = "ethercat_master");
		EthercatMaster(const EthercatMaster& other) = delete;
		EthercatMaster(EthercatMaster&& other) = delete;
		EthercatMaster& operator=(const EthercatMaster& other) = delete;
		EthercatMaster& operator=(EthercatMaster&& other) = delete;
		CODEIT_REGISTER_TYPE(EthercatMaster);

	protected:
		auto virtual send()->void override;
		auto virtual recv()->void override;
		auto virtual release()->void override;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;

		friend class PdoEntry;
	};
	class EthercatIO :public EthercatSlave, public virtual IO
	{
	public:
		auto isVirtual()const->bool;
		auto setVirtual(bool is_virtual = true)->void;
		auto eleNum()const->Size;
		auto setEleNum(Size num = 0)->void;
		auto virtual ioType()->IO_TYPE;
		auto virtual setIoType(IO_TYPE type_)->void;
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		EthercatIO(const std::string& name = "ethercat_io", std::uint16_t phy_id = 0
			, std::uint32_t vendor_id = 0x00000000, std::uint32_t product_code = 0x00000000, std::uint32_t revision_num = 0x00000000, std::uint32_t dc_assign_activate = 0x00000000
			, bool is_virtual = false);
		CODEIT_REGISTER_TYPE(EthercatIO);
		EthercatIO(const EthercatIO& other);
		EthercatIO(EthercatIO&& other) = delete;
		EthercatIO& operator=(const EthercatIO& other);
		EthercatIO& operator=(EthercatIO&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class EthercatDI :public EthercatIO
	{
	public:
		
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		auto virtual actualDi()->std::uint8_t;
	
		EthercatDI(const std::string& name = "ethercat_di", std::uint16_t phy_id = 0
			, std::uint32_t vendor_id = 0x00000000, std::uint32_t product_code = 0x00000000, std::uint32_t revision_num = 0x00000000, std::uint32_t dc_assign_activate = 0x00000000
			, bool is_virtual = false);
		CODEIT_REGISTER_TYPE(EthercatDI);
		EthercatDI(const EthercatDI& other);
		EthercatDI(EthercatDI&& other) = delete;
		EthercatDI& operator=(const EthercatDI& other);
		EthercatDI& operator=(EthercatDI&& other) = delete;
	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class EthercatDO :public EthercatIO
	{
	public:
		
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		auto virtual setDo(const std::uint8_t)->void;

		EthercatDO(const std::string& name = "ethercat_do", std::uint16_t phy_id = 0
			, std::uint32_t vendor_id = 0x00000000, std::uint32_t product_code = 0x00000000, std::uint32_t revision_num = 0x00000000, std::uint32_t dc_assign_activate = 0x00000000
			, bool is_virtual = false);
		CODEIT_REGISTER_TYPE(EthercatDO);
		EthercatDO(const EthercatDO& other);
		EthercatDO(EthercatDO&& other) = delete;
		EthercatDO& operator=(const EthercatDO& other);
		EthercatDO& operator=(EthercatDO&& other) = delete;
	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class EthercatAI :public EthercatIO
	{
	public:

		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		auto virtual actualAi()->vector<double>&;
		auto virtual resolution()const->double;
		auto virtual setResolution(double res_)->void;
		EthercatAI(const std::string& name = "ethercat_ai", std::uint16_t phy_id = 0
			, std::uint32_t vendor_id = 0x00000000, std::uint32_t product_code = 0x00000000, std::uint32_t revision_num = 0x00000000, std::uint32_t dc_assign_activate = 0x00000000
			, bool is_virtual = false);
		CODEIT_REGISTER_TYPE(EthercatAI);
		EthercatAI(const EthercatAI& other);
		EthercatAI(EthercatAI&& other) = delete;
		EthercatAI& operator=(const EthercatAI& other);
		EthercatAI& operator=(EthercatAI&& other) = delete;
	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class EthercatAO :public EthercatIO
	{
	public:

		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		auto virtual setAo(const vector<double>&)->void;
		auto virtual resolution()const->double;
		auto virtual setResolution(double res_)->void;
		EthercatAO(const std::string& name = "ethercat_ao", std::uint16_t phy_id = 0
			, std::uint32_t vendor_id = 0x00000000, std::uint32_t product_code = 0x00000000, std::uint32_t revision_num = 0x00000000, std::uint32_t dc_assign_activate = 0x00000000
			, bool is_virtual = false);
		CODEIT_REGISTER_TYPE(EthercatAO);
		EthercatAO(const EthercatAO& other);
		EthercatAO(EthercatAO&& other) = delete;
		EthercatAO& operator=(const EthercatAO& other);
		EthercatAO& operator=(EthercatAO&& other) = delete;
	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};


	class EthercatMotor :public EthercatSlave, public Motor
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
		// require pdo 0x606C //
		auto virtual actualVel()const->double override;
		// require pdo 0x6077 //
		auto virtual actualToq()const->double override;
		// require pdo 0x6078 //
		auto virtual actualCur()const->double override;
		// require pdo 0x2064 //
		auto virtual actualAddlPos()const->double override;
		auto virtual setErrorCode(std::int32_t code)->void override { ; }
		// require pdo 0x6040 0x6041 // 
		auto virtual disable()->int override;
		// require pdo 0x6040 0x6041 //
		auto virtual enable()->int override;
		// require pdo 0x6040 0x6041 0x6060 0x6061 //
		auto virtual home()->int override;
		// require pdo 0x6060 0x6061 //
		auto virtual mode(std::uint8_t md)->int override;

		auto virtual velDiff()const->double override;
		auto virtual setVelDiff(double vel)->void override;

		auto toqFrom6077()->bool;
		auto setTorFrom6077(bool flag)->void;

		EthercatMotor(const std::string& name = "ethercat_motion", std::uint16_t phy_id = 0
			, std::uint32_t vendor_id = 0x00000000, std::uint32_t product_code = 0x00000000, std::uint32_t revision_num = 0x00000000, std::uint32_t dc_assign_activate = 0x00000300
			, double max_pos = 1.0, double min_pos = -1.0, double max_vel = 1.0, double min_vel = -1.0, double max_acc = 1.0, double min_acc = -1.0
			, double max_pos_following_error = 1.0, double max_vel_following_error = 1.0, double pos_factor = 1.0, double pos_offset = 0.0, double home_pos = 0.0, double max_collision = 30.0, bool is_6077=true,bool is_virtual = false);
		CODEIT_REGISTER_TYPE(EthercatMotor);
		EthercatMotor(const EthercatMotor& other);
		EthercatMotor(EthercatMotor&& other) = delete;
		EthercatMotor& operator=(const EthercatMotor& other);
		EthercatMotor& operator=(EthercatMotor&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class EthercatExternMotor :public EthercatSlave, public ExternMotor
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
		// require pdo 0x606C //
		auto virtual actualVel()const->double override;
		// require pdo 0x6077 //
		auto virtual actualToq()const->double override;
		// require pdo 0x6078 //
		auto virtual actualCur()const->double override;
		// require pdo 0x2064 //
		//auto virtual actualAddlPos()const->double override;

		// require pdo 0x6040 0x6041 // 
		auto virtual disable()->int override;
		// require pdo 0x6040 0x6041 //
		auto virtual enable()->int override;
		// require pdo 0x6040 0x6041 0x6060 0x6061 //
		auto virtual home()->int override;
		// require pdo 0x6060 0x6061 //
		auto virtual mode(std::uint8_t md)->int override;

		EthercatExternMotor(const std::string& name = "ethercat_extern_motion", std::uint16_t phy_id = 0
			, std::uint32_t vendor_id = 0x00000000, std::uint32_t product_code = 0x00000000, std::uint32_t revision_num = 0x00000000, std::uint32_t dc_assign_activate = 0x00000300
			, double max_pos = 1.0, double min_pos = -1.0, double max_vel = 1.0, double min_vel = -1.0, double max_acc = 1.0, double min_acc = -1.0
			, double max_pos_following_error = 1.0, double max_vel_following_error = 1.0, double pos_factor = 1.0, double pos_offset = 0.0, double home_pos = 0.0, bool is_virtual = false);
		CODEIT_REGISTER_TYPE(EthercatExternMotor);
		EthercatExternMotor(const EthercatExternMotor& other);
		EthercatExternMotor(EthercatExternMotor&& other) = delete;
		EthercatExternMotor& operator=(const EthercatExternMotor& other);
		EthercatExternMotor& operator=(EthercatExternMotor&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class EthercatFTsensor :public EthercatSlave, public FTsensor
	{
	public:
		auto isVirtual()->bool;
		auto setVirtual(bool is_virtual = true)->void;

		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		
		auto virtual fx()const->double override;
		auto virtual fy()const->double override;
		auto virtual fz()const->double override;
		auto virtual mx()const->double override;
		auto virtual my()const->double override;
		auto virtual mz()const->double override;

		EthercatFTsensor(const std::string& name = "ethercat_motion", std::uint16_t phy_id = 0
			, std::uint32_t vendor_id = 0x00000000, std::uint32_t product_code = 0x00000000, std::uint32_t revision_num = 0x00000000, std::uint32_t dc_assign_activate = 0x00000000
			, double ratio = 1.0, double fx_offset = 0.0, double fy_offset = 0.0,
			double fz_offset = 0.0, double mx_offset = 0.0, double my_offset = 0.0, double mz_offset = 0.0, bool is_virtual = false);
		CODEIT_REGISTER_TYPE(EthercatFTsensor);
		EthercatFTsensor(const EthercatFTsensor& other);
		EthercatFTsensor(EthercatFTsensor&& other) = delete;
		EthercatFTsensor& operator=(const EthercatFTsensor& other);
		EthercatFTsensor& operator=(EthercatFTsensor&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	
	class EthercatController :public EthercatMaster, public Controller
	{
	public:
		using MotionPool = core::SubRefPool<EthercatMotor, core::ChildRefPool<EthercatSlave, core::ObjectPool<Slave>>>;
		auto motionPool()->MotionPool&;
		auto motionPool()const->const MotionPool& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionPool(); }
		auto motionAtAbs(Size id)->EthercatMotor& { return dynamic_cast<EthercatMotor&>(Controller::motionAtAbs(id)); }
		auto motionAtAbs(Size id)const->const EthercatMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionAtAbs(id); }
		auto motionAtPhy(Size id)->EthercatMotor& { return dynamic_cast<EthercatMotor&>(Controller::motionAtPhy(id)); }
		auto motionAtPhy(Size id)const->const EthercatMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionAtPhy(id); }
		auto motionAtSla(Size id)->EthercatMotor& { return dynamic_cast<EthercatMotor&>(Controller::motionAtSla(id)); }
		auto motionAtSla(Size id)const->const EthercatMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionAtSla(id); }

		using ExternMotionPool = core::SubRefPool<EthercatExternMotor, core::ChildRefPool<EthercatSlave, core::ObjectPool<Slave>>>;
		auto externMotionPool()->ExternMotionPool&;
		auto externMotionPool()const->const ExternMotionPool& { return const_cast<std::decay_t<decltype(*this)>*>(this)->externMotionPool(); }
		auto externMotionAtAbs(Size id)->EthercatExternMotor& { return dynamic_cast<EthercatExternMotor&>(Controller::externMotionAtAbs(id)); }
		auto externMotionAtAbs(Size id)const->const EthercatExternMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->externMotionAtAbs(id); }
		auto externMotionAtPhy(Size id)->EthercatExternMotor& { return dynamic_cast<EthercatExternMotor&>(Controller::externMotionAtPhy(id)); }
		auto externMotionAtPhy(Size id)const->const EthercatExternMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->externMotionAtPhy(id); }
		auto externMotionAtSla(Size id)->EthercatExternMotor& { return dynamic_cast<EthercatExternMotor&>(Controller::externMotionAtSla(id)); }
		auto externMotionAtSla(Size id)const->const EthercatExternMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->externMotionAtSla(id); }


		auto slavePool()->core::ChildRefPool<EthercatSlave, core::ObjectPool<Slave>>& { return EthercatMaster::slavePool(); }
		auto slavePool()const->const core::ChildRefPool<EthercatSlave, core::ObjectPool<Slave>>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slavePool(); }
		auto slaveAtAbs(Size id)->EthercatSlave& { return EthercatMaster::slaveAtAbs(id); }
		auto slaveAtAbs(Size id)const->const EthercatSlave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtAbs(id); }
		auto slaveAtPhy(Size id)->EthercatSlave& { return EthercatMaster::slaveAtPhy(id); }
		auto slaveAtPhy(Size id)const->const EthercatSlave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtPhy(id); }

		using IOPool = core::SubRefPool<EthercatIO, core::ChildRefPool<EthercatSlave, core::ObjectPool<Slave>>>;
		auto ioPool()->IOPool&;
		auto ioPool()const->const IOPool& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioPool(); }
		auto ioAtAbs(Size id)->EthercatIO& { return dynamic_cast<EthercatIO&>(Controller::ioAtAbs(id)); }
		auto ioAtAbs(Size id)const->const EthercatIO& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioAtAbs(id); }
		auto ioAtPhy(Size id)->EthercatIO& { return dynamic_cast<EthercatIO&>(Controller::ioAtPhy(id)); }
		auto ioAtPhy(Size id)const->const EthercatIO& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioAtPhy(id); }
		auto ioAtSla(Size id)->EthercatIO& { return dynamic_cast<EthercatIO&>(Controller::ioAtSla(id)); }
		auto ioAtSla(Size id)const->const EthercatIO& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioAtSla(id); }

		using FTsensorPool = core::SubRefPool<EthercatFTsensor, core::ChildRefPool<EthercatSlave, core::ObjectPool<Slave>>>;
		auto ftsensorPool()->FTsensorPool&;
		auto ftsensorPool()const->const FTsensorPool& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ftsensorPool(); }
		auto ftsensorAtAbs(Size id)->EthercatFTsensor& { return dynamic_cast<EthercatFTsensor&>(Controller::ftsensorAtAbs(id)); }
		auto ftsensorAtAbs(Size id)const->const EthercatFTsensor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ftsensorAtAbs(id); }
		auto ftsensorAtPhy(Size id)->EthercatFTsensor& { return dynamic_cast<EthercatFTsensor&>(Controller::ftsensorAtPhy(id)); }
		auto ftsensorAtPhy(Size id)const->const EthercatFTsensor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ftsensorAtPhy(id); }
		auto ftsensorAtSla(Size id)->EthercatFTsensor& { return dynamic_cast<EthercatFTsensor&>(Controller::ftsensorAtSla(id)); }
		auto ftsensorAtSla(Size id)const->const EthercatFTsensor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ftsensorAtSla(id); }


		auto virtual start()->void override { EthercatMaster::start(); }
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override { EthercatMaster::saveXml(xml_ele); }
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override { EthercatMaster::loadXml(xml_ele); }

		virtual ~EthercatController();
		EthercatController(const std::string& name = "ethercat_controller");
		EthercatController(const EthercatController& other) = delete;
		EthercatController(EthercatController&& other) = delete;
		EthercatController& operator=(const EthercatController& other) = delete;
		EthercatController& operator=(EthercatController&& other) = delete;
		CODEIT_REGISTER_TYPE(EthercatController);

	protected:
		auto virtual init()->void override;
		auto virtual send()->void override { EthercatMaster::send(); }
		auto virtual recv()->void override { EthercatMaster::recv(); }
		auto virtual release()->void override { EthercatMaster::release(); }

		struct Imp;
		core::ImpPtr<Imp> imp_;
	};



	
}

#endif