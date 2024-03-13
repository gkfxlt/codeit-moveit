#ifndef CODEIT_CONTROLLER_SOCK_TCP_H_
#define CODEIT_CONTROLLER_SOCK_TCP_H_

#include <filesystem>

#include <codeit/controller/master_slave.hpp>
#include<codeit/controller/io_device.hpp>
#include<codeit/controller/controller_motor.hpp>
#include<codeit/controller/master_slave.hpp>
#include <codeit/core/msg.hpp>

namespace codeit::controller
{
	class SocketSlave : virtual public Slave
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		virtual ~SocketSlave();
		explicit SocketSlave(const std::string& name = "sock_slave", std::uint16_t phy_id = 0);
		CODEIT_REGISTER_TYPE(SocketSlave);
		SocketSlave(const SocketSlave& other);
		SocketSlave(SocketSlave&& other) = delete;
		SocketSlave& operator=(const SocketSlave& other);
		SocketSlave& operator=(SocketSlave&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;

		friend class ComMaster;
	};

	class SocketMotor :public SocketSlave, public Motor
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

		SocketMotor(const std::string& name = "sock_motion", std::uint16_t phy_id = 0
			, std::uint32_t vendor_id = 0x00000000, std::uint32_t product_code = 0x00000000, std::uint32_t revision_num = 0x00000000, std::uint32_t dc_assign_activate = 0x00000000
			, double max_pos = 1.0, double min_pos = -1.0, double max_vel = 1.0, double min_vel = -1.0, double max_acc = 1.0, double min_acc = -1.0
			, double max_pos_following_error = 1.0, double max_vel_following_error = 1.0, double pos_factor = 1.0, double pos_offset = 0.0, double home_pos = 0.0, bool is_virtual = false);
		CODEIT_REGISTER_TYPE(SocketMotor);
		SocketMotor(const SocketMotor& other);
		SocketMotor(SocketMotor&& other) = delete;
		SocketMotor& operator=(const SocketMotor& other);
		SocketMotor& operator=(SocketMotor&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class SocketSubSystem :public SocketSlave, public SubSystem
	{
	
	public:
		auto isVirtual()const->bool;
		auto setVirtual(bool is_virtual = true)->void;
		auto virtual actualCmd()->std::string;
		auto virtual setCmd(std::string)->void;
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		SocketSubSystem(const std::string& name = "socket_subsystem", std::uint16_t phy_id = 0
			, bool is_virtual = false);
		CODEIT_REGISTER_TYPE(SocketSubSystem);
		SocketSubSystem(const SocketSubSystem& other)=delete ;
		SocketSubSystem(SocketSubSystem&& other)= delete;
		SocketSubSystem& operator=(const SocketSubSystem& other)=delete ;
		SocketSubSystem& operator=(SocketSubSystem&& other)=delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class SocketIO :public SocketSlave, public virtual IO
	{
	public:
		auto isVirtual()const->bool;
		auto setVirtual(bool is_virtual = true)->void;

		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		SocketIO(const std::string& name = "socket_io", std::uint16_t phy_id = 0
			, bool is_virtual = false);
		CODEIT_REGISTER_TYPE(SocketIO);
		SocketIO(const SocketIO& other);
		SocketIO(SocketIO&& other) = delete;
		SocketIO& operator=(const SocketIO& other);
		SocketIO& operator=(SocketIO&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};


	class SocketDI :public SocketIO, public DI
	{
	public:

		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		auto virtual actualDi()const->bool override;
		auto virtual setDi(bool)->void override;

		SocketDI(const std::string& name = "sock_di", std::uint16_t phy_id = 0, bool is_virtual = false);
		CODEIT_REGISTER_TYPE(SocketDI);
		SocketDI(const SocketDI& other) = delete;
		SocketDI(SocketDI&& other) = delete;
		SocketDI& operator=(const SocketDI& other) = delete;
		SocketDI& operator=(SocketDI&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class SocketDO :public SocketIO, public DO
	{
	public:

		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		auto virtual actualDo()const->bool override;
		auto virtual setDo(bool)->void override;

		SocketDO(const std::string& name = "sock_do", std::uint16_t phy_id = 0, bool is_virtual = false);
		CODEIT_REGISTER_TYPE(SocketDO);
		SocketDO(const SocketDO& other) = delete;
		SocketDO(SocketDO&& other) = delete;
		SocketDO& operator=(const SocketDO& other) = delete;
		SocketDO& operator=(SocketDO&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class SocketAI :public SocketIO, public AI
	{
	public:

		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		auto virtual actualAi()const->double override;
		auto virtual setAi(double)->void override;

		SocketAI(const std::string& name = "sock_ai", std::uint16_t phy_id = 0, bool is_virtual = false, double res_ = 1);
		CODEIT_REGISTER_TYPE(SocketAI);
		SocketAI(const SocketAI& other) = delete;
		SocketAI(SocketAI&& other) = delete;
		SocketAI& operator=(const SocketAI& other) = delete;
		SocketAI& operator=(SocketAI&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};

	class SocketAO :public SocketIO, public AO
	{
	public:

		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		auto virtual actualAo()const->double override;
		auto virtual setAo(double)->void override;

		SocketAO(const std::string& name = "sock_ao", std::uint16_t phy_id = 0, bool is_virtual = false, double res_ = 1);
		CODEIT_REGISTER_TYPE(SocketAO);
		SocketAO(const SocketAO& other) = delete;
		SocketAO(SocketAO&& other) = delete;
		SocketAO& operator=(const SocketAO& other) = delete;
		SocketAO& operator=(SocketAO&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};



	class SocketMaster : virtual public NrtMaster
	{
	public:

		auto virtual init()->void override;
		//auto virtual start()->void override;
		//auto virtual stop()->void override;
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		enum State
		{
			IDLE = 0,
			WAITING_FOR_CONNECTION,
			WORKING
		};
		enum TYPE
		{
			TCP,
			UDP,
			WEB,
			UDP_RAW,
			WEB_RAW
		};

	public:
		auto isConnected()->bool;
		auto state()->State;
		auto startServer(const std::string& port = std::string())->void;
		auto startClient(const std::string& port = std::string())->void;
		auto connect(const std::string& remote_ip = std::string(), const std::string& port = std::string())->void;
		
		//auto port()const->const std::string&;
		auto port() const->const std::string&;
		auto setPort(const std::string& port)->void;
		auto remoteIP()const->const std::string&;
		auto setRemoteIP(const std::string& remote_ip)->void;
		auto isServer()const->const bool&;
		auto setIsServer(const bool& is_server)->void;
		auto connectType()const->TYPE;
		auto setConnectType(const TYPE type)->void;
		auto setOnReceivedMsg(std::function<int(SocketMaster*, core::Msg&)> = nullptr)->void;
		auto setOnReceivedRawData(std::function<int(SocketMaster*, const char* data, int size)> = nullptr)->void;
		auto setOnReceivedConnection(std::function<int(SocketMaster*, const char* remote_ip, int remote_port)> = nullptr)->void;
		auto setOnLoseConnection(std::function<int(SocketMaster*)> = nullptr)->void;
		auto setOnGetConnection(std::function<int(SocketMaster*)> = nullptr)->void;
		////****ÐÂÔö**//
		auto setOnSendRawData(std::function<int(SocketMaster*)> = nullptr)->void;

		

		auto slavePool()->core::ChildRefPool<SocketSlave, core::ObjectPool<Slave>>&;
		auto slavePool()const->const core::ChildRefPool<SocketSlave, core::ObjectPool<Slave>>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slavePool(); }
		auto slaveAtAbs(Size id)->SocketSlave& { return dynamic_cast<SocketSlave&>(NrtMaster::slaveAtAbs(id)); }
		auto slaveAtAbs(Size id)const->const SocketSlave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtAbs(id); }
		auto slaveAtPhy(Size id)->SocketSlave& { return dynamic_cast<SocketSlave&>(NrtMaster::slaveAtPhy(id)); }
		auto slaveAtPhy(Size id)const->const SocketSlave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtPhy(id); }

		virtual ~SocketMaster();
		SocketMaster(const std::string& name = "sock_master", const std::string& remote_ip = "", const std::string& port = "", TYPE type = TYPE::TCP, const Size& packSize=0, const Size& nrt_id = 0, const bool& is_server=true);
		SocketMaster(const SocketMaster& other) = delete;
		SocketMaster(SocketMaster&& other) = delete;
		SocketMaster& operator=(const SocketMaster& other) = delete;
		SocketMaster& operator=(SocketMaster&& other) = delete;
		CODEIT_REGISTER_TYPE(SocketMaster);

	protected:
		auto virtual send()->void override;
		auto virtual recv()->void override;
		auto virtual release()->void override;

	private:
		struct Imp;
		const std::unique_ptr<Imp> imp_;

	};


	class SocketController :public SocketMaster, public NrtController
	{
	public:

		using MotionPool = core::SubRefPool<SocketMotor, core::ChildRefPool<SocketSlave, core::ObjectPool<Slave>>>;
		auto motionPool()->MotionPool&;
		auto motionPool()const->const MotionPool& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionPool(); }
		auto motionAtAbs(Size id)->SocketMotor& { return dynamic_cast<SocketMotor&>(NrtController::motionAtAbs(id)); }
		auto motionAtAbs(Size id)const->const SocketMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionAtAbs(id); }
		auto motionAtPhy(Size id)->SocketMotor& { return dynamic_cast<SocketMotor&>(NrtController::motionAtPhy(id)); }
		auto motionAtPhy(Size id)const->const SocketMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionAtPhy(id); }
		auto motionAtSla(Size id)->SocketMotor& { return dynamic_cast<SocketMotor&>(NrtController::motionAtSla(id)); }
		auto motionAtSla(Size id)const->const SocketMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionAtSla(id); }

		using IOPool = core::SubRefPool<SocketIO, core::ChildRefPool<SocketSlave, core::ObjectPool<Slave>>>;
		auto ioPool()->IOPool&;
		auto ioPool()const->const IOPool& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioPool(); }
		auto ioAtAbs(Size id)->SocketIO& { return dynamic_cast<SocketIO&>(NrtController::ioAtAbs(id)); }
		auto ioAtAbs(Size id)const->const SocketIO& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioAtAbs(id); }
		auto ioAtPhy(Size id)->SocketIO& { return dynamic_cast<SocketIO&>(NrtController::ioAtPhy(id)); }
		auto ioAtPhy(Size id)const->const SocketIO& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioAtPhy(id); }
		auto ioAtSla(Size id)->SocketIO& { return dynamic_cast<SocketIO&>(NrtController::ioAtSla(id)); }
		auto ioAtSla(Size id)const->const SocketIO& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioAtSla(id); }

		using SubSysPool = core::SubRefPool<SocketSubSystem, core::ChildRefPool<SocketSlave, core::ObjectPool<Slave>>>;
		auto subsysPool()->SubSysPool&;
		auto subsysPool()const->const SubSysPool& { return const_cast<std::decay_t<decltype(*this)>*>(this)->subsysPool(); }
		auto subsysAtAbs(Size id)->SocketSubSystem& { return dynamic_cast<SocketSubSystem&>(NrtController::subsysAtAbs(id)); }
		auto subsysAtAbs(Size id)const->const SocketSubSystem& { return const_cast<std::decay_t<decltype(*this)>*>(this)->subsysAtAbs(id); }
		auto subsysAtPhy(Size id)->SocketSubSystem& { return dynamic_cast<SocketSubSystem&>(NrtController::subsysAtPhy(id)); }
		auto subsysAtPhy(Size id)const->const SocketSubSystem& { return const_cast<std::decay_t<decltype(*this)>*>(this)->subsysAtPhy(id); }
		auto subsysAtSla(Size id)->SocketSubSystem& { return dynamic_cast<SocketSubSystem&>(NrtController::subsysAtSla(id)); }
		auto subsysAtSla(Size id)const->const SocketSubSystem& { return const_cast<std::decay_t<decltype(*this)>*>(this)->subsysAtSla(id); }


		auto virtual start()->void override { SocketMaster::start(); }
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override { SocketMaster::saveXml(xml_ele); }
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override { SocketMaster::loadXml(xml_ele); }

		auto slavePool()->core::ChildRefPool<SocketSlave, core::ObjectPool<Slave>>& { return SocketMaster::slavePool(); }
		auto slavePool()const->const core::ChildRefPool<SocketSlave, core::ObjectPool<Slave>>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slavePool(); }
		auto slaveAtAbs(Size id)->SocketSlave& { return SocketMaster::slaveAtAbs(id); }
		auto slaveAtAbs(Size id)const->const SocketSlave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtAbs(id); }
		auto slaveAtPhy(Size id)->SocketSlave& { return SocketMaster::slaveAtPhy(id); }
		auto slaveAtPhy(Size id)const->const SocketSlave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtPhy(id); }

		virtual ~SocketController();
		SocketController(const std::string& name = "socket_controller", const std::string& remote_ip = "", const std::string& port = "", TYPE type = TYPE::TCP, const Size& packSize=0, const Size& nrt_id = 0, const bool& is_server=true);
		SocketController(const SocketController& other) = delete;
		SocketController(SocketController&& other) = delete;
		SocketController& operator=(const SocketController& other) = delete;
		SocketController& operator=(SocketController&& other) = delete;
		CODEIT_REGISTER_TYPE(SocketController);

	protected:
		auto virtual init()->void override;
		auto virtual send()->void override { SocketMaster::send(); }
		auto virtual recv()->void override { SocketMaster::recv(); }
		auto virtual release()->void override { SocketMaster::release(); }

		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
}

#endif
