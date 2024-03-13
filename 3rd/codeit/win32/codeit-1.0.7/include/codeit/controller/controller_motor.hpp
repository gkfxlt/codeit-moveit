#ifndef CODEIT_CONTROLLER_MOTOR_H_
#define CODEIT_CONTROLLER_MOTOR_H_
#include <codeit/controller/master_slave.hpp>
#include <codeit/controller/io_device.hpp>
namespace codeit::controller
{

	class Motor : public virtual Slave
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto motId()const->Size;
		auto maxPos()const->double;
		auto setMaxPos(double max_pos)->void;
		auto minPos()const->double;
		auto setMinPos(double min_pos)->void;
		auto maxVel()const->double;
		auto setMaxVel(double max_vel)->void;
		auto minVel()const->double;
		auto setMinVel(double min_vel)->void;
		auto maxAcc()const->double;
		auto setMaxAcc(double max_acc)->void;
		auto minAcc()const->double;
		auto setMinAcc(double min_acc)->void;
		auto maxPosFollowingError()const->double;
		auto setMaxPosFollowingError(double max_pos_following_error)->void;
		auto maxVelFollowingError()const->double;
		auto setMaxVelFollowingError(double max_vel_following_error)->void;
		auto posOffset()const->double;
		auto setPosOffset(double pos_offset)->void;
		auto zeroOffset()const->double;
		auto setZeroOffset(double pos_offset)->void;
		auto torConst()const->double;
		auto setTorConst(double tor_const)->void;
		auto posFactor()const->double;
		auto setPosFactor(double pos_factor)->void;
		auto homePos()const->double;
		auto setHomePos(double home_pos)->void;
		auto maxCollision()const->double;
		auto setMaxCollision(double home_pos)->void;
		auto velFactor()const->double;
		auto setVelFactor(double home_pos)->void;
		auto posAddlFactor()const->double;
		auto setPosAddlFactor(double pos_factor)->void;
		auto posAddlOffset()const->double;
		auto setPosAddlOffset(double pos_offset)->void;
		auto toqOffsetFlag()const->bool;
		auto setToqOffsetFlag(bool flag=false)->void;
		auto velOffsetFlag()const->bool;
		auto setVelOffsetFlag(bool flag = false)->void;

		auto virtual controlWord()const->std::uint16_t = 0;
		auto virtual modeOfOperation()const->std::uint8_t = 0;
		auto virtual targetPos()const->double = 0;
		auto virtual targetVel()const->double = 0;
		auto virtual targetToq()const->double = 0;
		auto virtual offsetVel()const->double = 0;
		auto virtual offsetCur()const->double = 0;

		auto virtual setControlWord(std::uint16_t control_word)->void = 0;
		auto virtual setModeOfOperation(std::uint8_t mode)->void = 0;
		auto virtual setTargetPos(double pos)->void = 0;
		auto virtual setTargetVel(double vel)->void = 0;
		auto virtual setTargetToq(double toq)->void = 0;
		auto virtual setOffsetVel(double vel)->void = 0;
		auto virtual setOffsetToq(double toq)->void = 0;
		auto virtual setErrorCode(std::int32_t code)->void = 0;

		auto virtual statusWord()const->std::uint16_t = 0;
		auto virtual modeOfDisplay()const->std::uint8_t = 0;
		auto virtual errorCode()const->std::int32_t = 0;
		auto virtual actualPos()const->double = 0;
		auto virtual actualVel()const->double = 0;
		auto virtual actualToq()const->double = 0;
		auto virtual actualCur()const->double = 0;
		auto virtual actualAddlPos()const->double = 0;
		auto virtual velDiff()const->double = 0;
		auto virtual setVelDiff(double vel)->void = 0;


		auto virtual disable()->int = 0;
		auto virtual enable()->int = 0;
		auto virtual home()->int = 0;
		auto virtual mode(std::uint8_t md)->int = 0;

		virtual ~Motor();
		explicit Motor(const std::string& name = "motion", std::uint16_t phy_id = 0
			, double max_pos = 1.0, double min_pos = -1.0, double max_vel = 1.0, double min_vel = -1.0, double max_acc = 1.0, double min_acc = -1.0
			, double max_pos_following_error = 1.0, double max_vel_following_error = 1.0, double pos_factor = 1.0, double pos_offset = 0.0, double home_pos = 0.0, double max_collision=30.0, bool toq_offset_flag = false, bool vel_offset_flag = false);
		Motor(const Motor& other);
		Motor(Motor&& other) = delete;
		Motor& operator=(const Motor& other);
		Motor& operator=(Motor&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
		

		friend class Controller;
		friend class NrtController;
	};
	class ExternMotor : public virtual Slave
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto motId()const->Size;
		auto maxPos()const->double;
		auto setMaxPos(double max_pos)->void;
		auto minPos()const->double;
		auto setMinPos(double min_pos)->void;
		auto maxVel()const->double;
		auto setMaxVel(double max_vel)->void;
		auto minVel()const->double;
		auto setMinVel(double min_vel)->void;
		auto maxAcc()const->double;
		auto setMaxAcc(double max_acc)->void;
		auto minAcc()const->double;
		auto setMinAcc(double min_acc)->void;
		auto maxPosFollowingError()const->double;
		auto setMaxPosFollowingError(double max_pos_following_error)->void;
		auto maxVelFollowingError()const->double;
		auto setMaxVelFollowingError(double max_vel_following_error)->void;
		auto posOffset()const->double;
		auto setPosOffset(double pos_offset)->void;
		auto zeroOffset()const->double;
		auto setZeroOffset(double pos_offset)->void;
		auto torConst()const->double;
		auto setTorConst(double tor_const)->void;
		auto posFactor()const->double;
		auto setPosFactor(double pos_factor)->void;
		auto homePos()const->double;
		auto setHomePos(double home_pos)->void;

		auto virtual controlWord()const->std::uint16_t = 0;
		auto virtual modeOfOperation()const->std::uint8_t = 0;
		auto virtual targetPos()const->double = 0;
		auto virtual targetVel()const->double = 0;
		auto virtual targetToq()const->double = 0;
		auto virtual offsetVel()const->double = 0;
		auto virtual offsetCur()const->double = 0;

		auto virtual setControlWord(std::uint16_t control_word)->void = 0;
		auto virtual setModeOfOperation(std::uint8_t mode)->void = 0;
		auto virtual setTargetPos(double pos)->void = 0;
		auto virtual setTargetVel(double vel)->void = 0;
		auto virtual setTargetToq(double toq)->void = 0;
		auto virtual setOffsetVel(double vel)->void = 0;
		auto virtual setOffsetToq(double toq)->void = 0;

		auto virtual statusWord()const->std::uint16_t = 0;
		auto virtual modeOfDisplay()const->std::uint8_t = 0;
		auto virtual actualPos()const->double = 0;
		auto virtual actualVel()const->double = 0;
		auto virtual actualToq()const->double = 0;
		auto virtual actualCur()const->double = 0;

		auto virtual disable()->int = 0;
		auto virtual enable()->int = 0;
		auto virtual home()->int = 0;
		auto virtual mode(std::uint8_t md)->int = 0;

		virtual ~ExternMotor();
		explicit ExternMotor(const std::string& name = "extern_motion", std::uint16_t phy_id = 0
			, double max_pos = 1.0, double min_pos = -1.0, double max_vel = 1.0, double min_vel = -1.0, double max_acc = 1.0, double min_acc = -1.0
			, double max_pos_following_error = 1.0, double max_vel_following_error = 1.0, double pos_factor = 1.0, double pos_offset = 0.0, double home_pos = 0.0);
		ExternMotor(const ExternMotor& other);
		ExternMotor(ExternMotor&& other) = delete;
		ExternMotor& operator=(const ExternMotor& other);
		ExternMotor& operator=(ExternMotor&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;


		friend class Controller;
		friend class NrtController;
	};
	class FTsensor : public virtual Slave
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto sensorId()const->Size;
		auto ratio()const->double;
		auto setRatio(double max_pos)->void;
		auto offsetFx()const->double;
		auto setOffsetFx(double offset)->void;
		auto offsetFy()const->double;
		auto setOffsetFy(double offset)->void; 
		auto offsetFz()const->double;
		auto setOffsetFz(double offset)->void;
		auto offsetMx()const->double;
		auto setOffsetMx(double offset)->void;
		auto offsetMy()const->double;
		auto setOffsetMy(double offset)->void;
		auto offsetMz()const->double;
		auto setOffsetMz(double offset)->void;

		auto virtual fx()const->double = 0;
		auto virtual fy()const->double = 0;
		auto virtual fz()const->double = 0;
		auto virtual mx()const->double = 0;
		auto virtual my()const->double = 0;
		auto virtual mz()const->double = 0;

		virtual ~FTsensor();
		explicit FTsensor(const std::string& name = "ft_sensor", std::uint16_t phy_id = 0
			, double ratio = 1.0, double fx_offset=0.0, double fy_offset=0.0, 
			double fz_offset=0.0, double mx_offset=0.0, double my_offset=0.0, double mz_offset=0.0 );
		FTsensor(const FTsensor& other);
		FTsensor(FTsensor&& other) = delete;
		FTsensor& operator=(const FTsensor& other);
		FTsensor& operator=(FTsensor&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
		friend class Controller;
	};
	class Controller : public virtual Master
	{
	public:
		auto virtual init()->void override;
		auto motionPool()->core::SubRefPool<Motor, core::ObjectPool<Slave>>&;
		auto motionPool()const->const core::SubRefPool<Motor, core::ObjectPool<Slave>>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionPool(); }
		auto motionAtSla(Size id)->Motor&;
		auto motionAtSla(Size id)const->const Motor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionAtSla(id); }
		auto motionAtPhy(Size id)->Motor&;
		auto motionAtPhy(Size id)const->const Motor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionAtPhy(id); }
		auto motionAtAbs(Size id)->Motor&;
		auto motionAtAbs(Size id)const->const Motor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionAtAbs(id); }

		auto externMotionPool()->core::SubRefPool<ExternMotor, core::ObjectPool<Slave>>&;
		auto externMotionPool()const->const core::SubRefPool<ExternMotor, core::ObjectPool<Slave>>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->externMotionPool(); }
		auto externMotionAtSla(Size id)->ExternMotor&;
		auto externMotionAtSla(Size id)const->const ExternMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->externMotionAtSla(id); }
		auto externMotionAtPhy(Size id)->ExternMotor&;
		auto externMotionAtPhy(Size id)const->const ExternMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->externMotionAtPhy(id); }
		auto externMotionAtAbs(Size id)->ExternMotor&;
		auto externMotionAtAbs(Size id)const->const ExternMotor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->externMotionAtAbs(id); }

		auto ioPool()->core::SubRefPool<IO, core::ObjectPool<Slave>>&;
		auto ioPool()const->const core::SubRefPool<IO, core::ObjectPool<Slave>>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioPool(); }
		auto ioAtSla(Size id)->IO&;
		auto ioAtSla(Size id)const->const IO& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioAtSla(id); }
		auto ioAtPhy(Size id)->IO&;
		auto ioAtPhy(Size id)const->const IO& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioAtPhy(id); }
		auto ioAtAbs(Size id)->IO&;
		auto ioAtAbs(Size id)const->const IO& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioAtAbs(id); }

		auto ftsensorPool()->core::SubRefPool<FTsensor, core::ObjectPool<Slave>>&;
		auto ftsensorPool()const->const core::SubRefPool<FTsensor, core::ObjectPool<Slave>>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ftsensorPool(); }
		auto ftsensorAtSla(Size id)->FTsensor&;
		auto ftsensorAtSla(Size id)const->const FTsensor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ftsensorAtSla(id); }
		auto ftsensorAtPhy(Size id)->FTsensor&;
		auto ftsensorAtPhy(Size id)const->const FTsensor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ftsensorAtPhy(id); }
		auto ftsensorAtAbs(Size id)->FTsensor&;
		auto ftsensorAtAbs(Size id)const->const FTsensor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ftsensorAtAbs(id); }



		CODEIT_REGISTER_TYPE(Controller);
		virtual ~Controller();
		Controller(const std::string& name = "controller");
		Controller(const Controller& other) = delete;
		Controller(Controller&& other) = delete;
		Controller& operator=(const Controller& other) = delete;
		Controller& operator=(Controller&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};


	///***新增NrtController**//
	class NrtController : public virtual NrtMaster
	{
	public:
		auto virtual init()->void override;
		auto motionPool()->core::SubRefPool<Motor, core::ObjectPool<Slave>>&;
		auto motionPool()const->const core::SubRefPool<Motor, core::ObjectPool<Slave>>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionPool(); }
		auto motionAtSla(Size id)->Motor&;
		auto motionAtSla(Size id)const->const Motor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionAtSla(id); }
		auto motionAtPhy(Size id)->Motor&;
		auto motionAtPhy(Size id)const->const Motor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionAtPhy(id); }
		auto motionAtAbs(Size id)->Motor&;
		auto motionAtAbs(Size id)const->const Motor& { return const_cast<std::decay_t<decltype(*this)>*>(this)->motionAtAbs(id); }

		auto ioPool()->core::SubRefPool<IO, core::ObjectPool<Slave>>&;
		auto ioPool()const->const core::SubRefPool<IO, core::ObjectPool<Slave>>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioPool(); }
		auto ioAtSla(Size id)->IO&;
		auto ioAtSla(Size id)const->const IO& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioAtSla(id); }
		auto ioAtPhy(Size id)->IO&;
		auto ioAtPhy(Size id)const->const IO& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioAtPhy(id); }
		auto ioAtAbs(Size id)->IO&;
		auto ioAtAbs(Size id)const->const IO& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioAtAbs(id); }

		auto subsysPool()->core::SubRefPool<SubSystem, core::ObjectPool<Slave>>&;
		auto subsysPool()const->const core::SubRefPool<SubSystem, core::ObjectPool<Slave>>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->subsysPool(); }
		auto subsysAtSla(Size id)->SubSystem&;
		auto subsysAtSla(Size id)const->const SubSystem& { return const_cast<std::decay_t<decltype(*this)>*>(this)->subsysAtSla(id); }
		auto subsysAtPhy(Size id)->SubSystem&;
		auto subsysAtPhy(Size id)const->const SubSystem& { return const_cast<std::decay_t<decltype(*this)>*>(this)->subsysAtPhy(id); }
		auto subsysAtAbs(Size id)->SubSystem&;
		auto subsysAtAbs(Size id)const->const SubSystem& { return const_cast<std::decay_t<decltype(*this)>*>(this)->subsysAtAbs(id); }


		//auto logFile(const char* file_name) { NrtMaster::logFile(file_name); }


		CODEIT_REGISTER_TYPE(NrtController);
		virtual ~NrtController();
		NrtController(const std::string& name = "nrt_controller", const Size& packSize = 0, const Size& nrt_id=0);
		NrtController(const NrtController& other) = delete;
		NrtController(NrtController&& other) = delete;
		NrtController& operator=(const NrtController& other) = delete;
		NrtController& operator=(NrtController&& other) = delete;

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};

	


}

#endif
