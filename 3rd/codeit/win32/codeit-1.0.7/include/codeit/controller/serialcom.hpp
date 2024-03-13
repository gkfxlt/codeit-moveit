#ifndef CODEIT_CONTROLLER_SERIAL_COM_H_
#define CODEIT_CONTROLLER_SERIAL_COM_H_

#include <filesystem>

#include <codeit/controller/master_slave.hpp>
#include<codeit/controller/io_device.hpp>
#include<codeit/controller/controller_motor.hpp>
#include<codeit/controller/master_slave.hpp>
#include<codeit/core/serialport.hpp>
namespace codeit::controller
{
	class ComSlave : virtual public Slave
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		virtual ~ComSlave();
		explicit ComSlave(const std::string& name = "com_slave", std::uint16_t phy_id = 0);
		CODEIT_REGISTER_TYPE(ComSlave);
		ComSlave(const ComSlave& other);
		ComSlave(ComSlave&& other) = delete;
		ComSlave& operator=(const ComSlave& other);
		ComSlave& operator=(ComSlave&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;

		friend class ComMaster;
	};
	class ComMotor :public ComSlave, public Motor
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

		ComMotor(const std::string& name = "com_motion", std::uint16_t phy_id = 0
			, std::uint32_t vendor_id = 0x00000000, std::uint32_t product_code = 0x00000000, std::uint32_t revision_num = 0x00000000, std::uint32_t dc_assign_activate = 0x00000000
			, double max_pos = 1.0, double min_pos = -1.0, double max_vel = 1.0, double min_vel = -1.0, double max_acc = 1.0, double min_acc = -1.0
			, double max_pos_following_error = 1.0, double max_vel_following_error = 1.0, double pos_factor = 1.0, double pos_offset = 0.0, double home_pos = 0.0, bool is_virtual = false);
		CODEIT_REGISTER_TYPE(ComMotor);
		ComMotor(const ComMotor& other);
		ComMotor(ComMotor&& other) = delete;
		ComMotor& operator=(const ComMotor& other);
		ComMotor& operator=(ComMotor&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class ComSubSystem :public ComSlave, public SubSystem
	{

	public:
		auto isVirtual()const->bool;
		auto setVirtual(bool is_virtual = true)->void;
		auto virtual actualCmd()->std::string;
		auto virtual setCmd(std::string)->void;
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		ComSubSystem(const std::string& name = "com_subsystem", std::uint16_t phy_id = 0
			, bool is_virtual = false);
		CODEIT_REGISTER_TYPE(ComSubSystem);
		ComSubSystem(const ComSubSystem& other) = delete;
		ComSubSystem(ComSubSystem&& other) = delete;
		ComSubSystem& operator=(const ComSubSystem& other) = delete;
		ComSubSystem& operator=(ComSubSystem&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class ComIO :public ComSlave, public virtual IO
	{
	public:
		auto isVirtual()const->bool;
		auto setVirtual(bool is_virtual = true)->void;

		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		ComIO(const std::string& name = "com_io", std::uint16_t phy_id = 0
			, bool is_virtual = false);
		CODEIT_REGISTER_TYPE(ComIO);
		ComIO(const ComIO& other);
		ComIO(ComIO&& other) = delete;
		ComIO& operator=(const ComIO& other);
		ComIO& operator=(ComIO&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};

	class ComDI :public ComIO, public DI
	{
	public:
		
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		auto virtual actualDi()const->bool override;
		auto virtual setDi(bool)->void override;

		ComDI(const std::string& name = "com_di", std::uint16_t phy_id=0, bool is_virtual=false);
		CODEIT_REGISTER_TYPE(ComDI);
		ComDI(const ComDI& other) = delete;
		ComDI(ComDI&& other) = delete;
		ComDI& operator=(const ComDI& other) = delete;
		ComDI& operator=(ComDI&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class ComDO :public ComIO, public DO
	{
	public:
		
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		auto virtual actualDo()const->bool override;
		auto virtual setDo(bool)->void override;

		ComDO(const std::string& name = "com_do", std::uint16_t phy_id = 0, bool is_virtual = false);
		CODEIT_REGISTER_TYPE(ComDO);
		ComDO(const ComDO& other) = delete;
		ComDO(ComDO&& other) = delete;
		ComDO& operator=(const ComDO& other) = delete;
		ComDO& operator=(ComDO&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class ComAI :public ComIO, public AI
	{
	public:
		
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		auto virtual actualAi()const->double override;
		auto virtual setAi(double)->void override;

		ComAI(const std::string& name = "com_ai", std::uint16_t phy_id = 0, bool is_virtual = false, double resolution=1);
		CODEIT_REGISTER_TYPE(ComAI);
		ComAI(const ComAI& other) = delete;
		ComAI(ComAI&& other) = delete;
		ComAI& operator=(const ComAI& other) = delete;
		ComAI& operator=(ComAI&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class ComAO :public ComIO, public AO
	{
	public:
		
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		auto virtual actualAo()const->double override;
		auto virtual setAo(double)->void override;

		ComAO(const std::string& name = "com_ao", std::uint16_t phy_id = 0, bool is_virtual = false, double resolution = 1);
		CODEIT_REGISTER_TYPE(ComAO);
		ComAO(const ComAO& other) = delete;
		ComAO(ComAO&& other) = delete;
		ComAO& operator=(const ComAO& other) = delete;
		ComAO& operator=(ComAO&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};



	class ComMaster : virtual public NrtMaster
	{
	public:

		auto virtual init()->void override;
		//auto virtual start()->void override;
		//auto virtual stop()->void override;
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		bool WriteData(char* pData, Size length);
		bool ReadChar(char& cRecved);


#ifdef WIN32
		auto portNo()const->const UINT&;
		auto setPortNo(const UINT&)->void;
		auto baud()const->const UINT&;
		auto setBaud(const UINT&)->void;
		auto parity()const->const char&;
		auto setParity(const char&)->void;
		auto databits()const->const UINT&;
		auto setDatabits(const UINT&)->void;
		auto stopsbits()const->const UINT&;
		auto setStopsbits(const UINT&)->void;
		auto dwCommEvents()const->const DWORD&;
		auto setDwCommEvents(const DWORD&)->void;
		bool openPort(UINT  portNo);
		void ClosePort();
		bool InitPort(UINT  portNo, UINT  baud, char  parity, UINT  databits, UINT  stopsbits, DWORD dwCommEvents);
		bool InitPort(UINT  portNo, const LPDCB& plDCB);
		
		UINT GetBytesInCOM();
#endif

#ifdef UNIX
		bool open();
		bool open(const std::string& path, const core::SerialPort::ComOptions& options);
		void termiosOptions(termios& tios, const core::SerialPort::ComOptions& options);
		bool isOpen() const;
		auto portNo()const->const std::string&;
		auto baud()const->const core::SerialPort::BaudRate&;
		void ClosePort();
#endif    

		auto slavePool()->core::ChildRefPool<ComSlave, core::ObjectPool<Slave>>&;
		auto slavePool()const->const core::ChildRefPool<ComSlave, core::ObjectPool<Slave>>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slavePool(); }
		auto slaveAtAbs(Size id)->ComSlave& { return dynamic_cast<ComSlave&>(NrtMaster::slaveAtAbs(id)); }
		auto slaveAtAbs(Size id)const->const ComSlave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtAbs(id); }
		auto slaveAtPhy(Size id)->ComSlave& { return dynamic_cast<ComSlave&>(NrtMaster::slaveAtPhy(id)); }
		auto slaveAtPhy(Size id)const->const ComSlave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtPhy(id); }

		virtual ~ComMaster();
		ComMaster(const std::string& name = "com_master", const core::SerialPort::ComOptions& options = core::SerialPort::defaultComOptions, const Size& pack_size = 0, const Size& nrt_id = 0);
		ComMaster(const ComMaster& other) = delete;
		ComMaster(ComMaster&& other) = delete;
		ComMaster& operator=(const ComMaster& other) = delete;
		ComMaster& operator=(ComMaster&& other) = delete;
		CODEIT_REGISTER_TYPE(ComMaster);

	protected:
		auto virtual send()->void override;
		auto virtual recv()->void override;
		auto virtual release()->void override;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
		
	};

	
	class ComController :public ComMaster, public NrtController
	{
	public:
		using MotionPool = core::SubRefPool<ComMotor, core::ChildRefPool<ComSlave, core::ObjectPool<Slave>>>;
		auto motionPool()->MotionPool&;
		auto motionPool()const->const MotionPool& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionPool(); }
		auto motionAtAbs(Size id)->ComMotor& { return dynamic_cast<ComMotor&>(NrtController::motionAtAbs(id)); }
		auto motionAtAbs(Size id)const->const ComMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionAtAbs(id); }
		auto motionAtPhy(Size id)->ComMotor& { return dynamic_cast<ComMotor&>(NrtController::motionAtPhy(id)); }
		auto motionAtPhy(Size id)const->const ComMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionAtPhy(id); }
		auto motionAtSla(Size id)->ComMotor& { return dynamic_cast<ComMotor&>(NrtController::motionAtSla(id)); }
		auto motionAtSla(Size id)const->const ComMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionAtSla(id); }
		
		using SubSysPool = core::SubRefPool<ComSubSystem, core::ChildRefPool<ComSlave, core::ObjectPool<Slave>>>;
		auto subsysPool()->SubSysPool&;
		auto subsysPool()const->const SubSysPool& { return const_cast<std::decay_t<decltype(*this)>*>(this)->subsysPool(); }
		auto subsysAtAbs(Size id)->ComSubSystem& { return dynamic_cast<ComSubSystem&>(NrtController::subsysAtAbs(id)); }
		auto subsysAtAbs(Size id)const->const ComSubSystem& { return const_cast<std::decay_t<decltype(*this)>*>(this)->subsysAtAbs(id); }
		auto subsysAtPhy(Size id)->ComSubSystem& { return dynamic_cast<ComSubSystem&>(NrtController::subsysAtPhy(id)); }
		auto subsysAtPhy(Size id)const->const ComSubSystem& { return const_cast<std::decay_t<decltype(*this)>*>(this)->subsysAtPhy(id); }
		auto subsysAtSla(Size id)->ComSubSystem& { return dynamic_cast<ComSubSystem&>(NrtController::subsysAtSla(id)); }
		auto subsysAtSla(Size id)const->const ComSubSystem& { return const_cast<std::decay_t<decltype(*this)>*>(this)->subsysAtSla(id); }

		
		using IOPool = core::SubRefPool<ComIO, core::ChildRefPool<ComSlave, core::ObjectPool<Slave>>>;
		auto ioPool()->IOPool&;
		auto ioPool()const->const IOPool& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioPool(); }
		auto ioAtAbs(Size id)->ComIO& { return dynamic_cast<ComIO&>(NrtController::ioAtAbs(id)); }
		auto ioAtAbs(Size id)const->const ComIO& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioAtAbs(id); }
		auto ioAtPhy(Size id)->ComIO& { return dynamic_cast<ComIO&>(NrtController::ioAtPhy(id)); }
		auto ioAtPhy(Size id)const->const ComIO& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioAtPhy(id); }
		auto ioAtSla(Size id)->ComIO& { return dynamic_cast<ComIO&>(NrtController::ioAtSla(id)); }
		auto ioAtSla(Size id)const->const ComIO& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioAtSla(id); }

		
		
		auto virtual start()->void override { ComMaster::start(); }
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override { ComMaster::saveXml(xml_ele); }
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override { ComMaster::loadXml(xml_ele); }

		auto slavePool()->core::ChildRefPool<ComSlave, core::ObjectPool<Slave>>& { return ComMaster::slavePool(); }
		auto slavePool()const->const core::ChildRefPool<ComSlave, core::ObjectPool<Slave>>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slavePool(); }
		auto slaveAtAbs(Size id)->ComSlave& { return ComMaster::slaveAtAbs(id); }
		auto slaveAtAbs(Size id)const->const ComSlave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtAbs(id); }
		auto slaveAtPhy(Size id)->ComSlave& { return ComMaster::slaveAtPhy(id); }
		auto slaveAtPhy(Size id)const->const ComSlave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtPhy(id); }
	
		virtual ~ComController();

		ComController(const std::string& name = "com_controller", const core::SerialPort::ComOptions& options = core::SerialPort::defaultComOptions, const Size& pack_size = 0,const Size& nrt_id = 0);
		ComController(const ComController& other) = delete;
		ComController(ComController&& other) = delete;
		ComController& operator=(const ComController& other) = delete;
		ComController& operator=(ComController&& other) = delete;
		CODEIT_REGISTER_TYPE(ComController);

	protected:
		auto virtual init()->void override;
		auto virtual send()->void override { ComMaster::send(); }
		auto virtual recv()->void override { ComMaster::recv(); }
		auto virtual release()->void override { ComMaster::release(); }

		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
}

#endif
