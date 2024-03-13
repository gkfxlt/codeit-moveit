#ifndef CODEIT_SYSTEM_FUNCTION_H_
#define CODEIT_SYSTEM_FUNCTION_H_
#include<codeit/function/basefunc.hpp>
#include<codeit/system/interface.hpp>

namespace codeit::system { class ControlSystem; }
namespace codeit::function {

	/// \brief 清理错误
		///
		/// 
	class Clear : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;

		virtual ~Clear() = default;
		explicit Clear(const std::string& name = "clear");
		CODEIT_REGISTER_TYPE(Clear);
		CODEIT_DEFINE_BIG_FOUR(Clear);
	};

	/// \brief 实时循环内休息指定时间
		/// 
		///
		/// ### 参数定义 ###
		/// 指定休息时间，单位是count（1ms），默认为1000：
		/// + 休息5000ms：“sl -c=5000” 或 “sl --count=5000”
	class Sleep : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;

		virtual ~Sleep();
		explicit Sleep(const std::string& name = "sleep");
		CODEIT_REGISTER_TYPE(Sleep);
		CODEIT_DECLARE_BIG_FOUR(Sleep);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};


	class SetProgramRate :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);

		virtual ~SetProgramRate();
		explicit SetProgramRate(const std::string& name = "SetProgramRate");
		CODEIT_REGISTER_TYPE(SetProgramRate);
		CODEIT_DECLARE_BIG_FOUR(SetProgramRate);
	};

	class SaveXml : public BasisFunc {
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		explicit SaveXml(const std::string& name = "SaveXml_plan");
		CODEIT_REGISTER_TYPE(SaveXml);
	};

	class CSstart :public BasisFunc {
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;

		virtual ~CSstart();
		explicit CSstart(const std::string& name = "CSstart");
		CODEIT_REGISTER_TYPE(CSstart);
	};

	class CSstop :public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;

		virtual ~CSstop();
		explicit CSstop(const std::string& name = "CSstop");
		CODEIT_REGISTER_TYPE(CSstop);
	};


	//class NRTstart :public BasisFunc {
	//public:
	//	auto virtual prepareNrt(BasisFunc&, int)->void;
	//	virtual ~NRTstart();
	//	explicit NRTstart(const std::string& name = "NRTstart");
	//	CODEIT_REGISTER_TYPE(NRTstart);
	//};
	//class NRTstop :public BasisFunc
	//{
	//public:
	//	auto virtual prepareNrt(BasisFunc&, int)->void;
	//	virtual ~NRTstop();
	//	explicit NRTstop(const std::string& name = "NRTstop");
	//	CODEIT_REGISTER_TYPE(NRTstop);
	//};

	class Stop :public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		virtual int executeRT(BasisFunc&, int) { return 0; }
		virtual void collectNrt(BasisFunc&, int);
		virtual ~Stop();
		explicit Stop(const std::string& name = "Stop");
		CODEIT_REGISTER_TYPE(Stop);
	};

	class Pause :public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;

		virtual ~Pause();
		explicit Pause(const std::string& name = "Pause");
		CODEIT_REGISTER_TYPE(Pause);
	};

	class Start :public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;

		virtual ~Start();
		explicit Start(const std::string& name = "Start");
		CODEIT_REGISTER_TYPE(Start);
	};

	class ClearFuncPool :public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;

		virtual ~ClearFuncPool();
		explicit ClearFuncPool(const std::string& name = "ClearFuncPool");
		CODEIT_REGISTER_TYPE(ClearFuncPool);
	};

	class OpenInterface :public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;

		virtual ~OpenInterface();
		explicit OpenInterface(const std::string& name = "OpenInterface");
		CODEIT_REGISTER_TYPE(OpenInterface);
	};
	class CloseInterface :public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;

		virtual ~CloseInterface();
		explicit CloseInterface(const std::string& name = "CloseInterface");
		CODEIT_REGISTER_TYPE(CloseInterface);
	};
	class Get : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual collectNrt(BasisFunc&, int)->void;
		virtual ~Get();
		explicit Get(const std::string& name = "Get");
		CODEIT_REGISTER_TYPE(Get);
	};

	class GetMotionState : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		virtual ~GetMotionState();
		explicit GetMotionState(const std::string& name = "GetMotionState");
		CODEIT_REGISTER_TYPE(GetMotionState);
		CODEIT_DECLARE_BIG_FOUR(GetMotionState);
	};

	class GetRate : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		virtual ~GetRate();
		explicit GetRate(const std::string& name = "GetRate");
		CODEIT_REGISTER_TYPE(GetRate);
		CODEIT_DECLARE_BIG_FOUR(GetRate);
	};

	class GetBreakRate : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		virtual ~GetBreakRate();
		explicit GetBreakRate(const std::string& name = "GetBreakRate");
		CODEIT_REGISTER_TYPE(GetBreakRate);
		CODEIT_DECLARE_BIG_FOUR(GetBreakRate);
	};

	class TeachPoint : public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);

		virtual ~TeachPoint();
		explicit TeachPoint(const std::string& name = "TeachPoint");
		CODEIT_REGISTER_TYPE(TeachPoint);
		CODEIT_DECLARE_BIG_FOUR(TeachPoint);
	};

	class DeletePoint :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);
		virtual ~DeletePoint();
		explicit DeletePoint(const std::string& name = "DeletePoint");
		CODEIT_REGISTER_TYPE(DeletePoint);
		CODEIT_DECLARE_BIG_FOUR(DeletePoint);
	};

	class DefineParam :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);

		virtual ~DefineParam();
		explicit DefineParam(const std::string& name = "DefineParam");
		CODEIT_REGISTER_TYPE(DefineParam);
		CODEIT_DECLARE_BIG_FOUR(DefineParam);
	};

	class DeleteParam :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);

		virtual ~DeleteParam();
		explicit DeleteParam(const std::string& name = "DeleteParam");
		CODEIT_REGISTER_TYPE(DeleteParam);
		CODEIT_DECLARE_BIG_FOUR(DeleteParam);
	};

	class RenameParam :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);

		virtual ~RenameParam();
		explicit RenameParam(const std::string& name = "RenameParam");
		CODEIT_REGISTER_TYPE(RenameParam);
		CODEIT_DECLARE_BIG_FOUR(RenameParam);
	};

	class RevalueParam :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);

		virtual ~RevalueParam();
		explicit RevalueParam(const std::string& name = "RevalueParam");
		CODEIT_REGISTER_TYPE(RevalueParam);
		CODEIT_DECLARE_BIG_FOUR(RevalueParam);
	};

	class GetGravity :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);

		virtual ~GetGravity();
		explicit GetGravity(const std::string& name = "GetGravity");
		CODEIT_REGISTER_TYPE(GetGravity);
		CODEIT_DECLARE_BIG_FOUR(GetGravity);
	};

	class SetInstallAng :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);

		virtual ~SetInstallAng();
		explicit SetInstallAng(const std::string& name = "SetInstallAng");
		CODEIT_REGISTER_TYPE(SetInstallAng);
		CODEIT_DECLARE_BIG_FOUR(SetInstallAng);
	};

	class DeleteVar :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);
		virtual ~DeleteVar();
		explicit DeleteVar(const std::string& name = "DeleteVar");
		CODEIT_REGISTER_TYPE(DeleteVar);
		CODEIT_DECLARE_BIG_FOUR(DeleteVar);
	};

	class SetDH : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;
		explicit SetDH(const std::string& name = "SetDH_plan");
		virtual ~SetDH();
		CODEIT_REGISTER_TYPE(SetDH);
		CODEIT_DECLARE_BIG_FOUR(SetDH);
	};

	class SetDriver : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;
		explicit SetDriver(const std::string& name = "SetDriver_plan");
		virtual ~SetDriver();
		CODEIT_REGISTER_TYPE(SetDriver);
		CODEIT_DECLARE_BIG_FOUR(SetDriver);
	};

	class ScanSlave : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;
		explicit ScanSlave(const std::string& name = "ScanSlave_plan");
		virtual ~ScanSlave();
		CODEIT_REGISTER_TYPE(ScanSlave);
		CODEIT_DECLARE_BIG_FOUR(ScanSlave);
	};

	class GetXml :public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;

		virtual ~GetXml();
		explicit GetXml(const std::string& name = "GetXml");
		CODEIT_REGISTER_TYPE(GetXml);
		CODEIT_DECLARE_BIG_FOUR(GetXml);
	};

	class SetXml :public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;

		virtual ~SetXml();
		explicit SetXml(const std::string& name = "SetXml");
		CODEIT_REGISTER_TYPE(SetXml);
		CODEIT_DECLARE_BIG_FOUR(SetXml);
	};

	/// \brief 非实时prepareNRT内休息指定时间
		/// 当休息时间够了后，会自动启动pause指令
		/// 但运动
		/// ### 参数定义 ### 
	class Wait : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		virtual ~Wait();
		explicit Wait(const std::string& name = "Wait");
		CODEIT_REGISTER_TYPE(Wait);
		CODEIT_DECLARE_BIG_FOUR(Wait);

	};

	class Var : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc& plan_next, int)->void;
		explicit Var(const std::string& name = "Var");
		virtual ~Var();
		CODEIT_REGISTER_TYPE(Var);
		CODEIT_DECLARE_BIG_FOUR(Var);
	};

	class ReVarValue : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc& plan_next, int)->void;
		explicit ReVarValue(const std::string& name = "ReVarValue");
		virtual ~ReVarValue();
		CODEIT_REGISTER_TYPE(ReVarValue);
		CODEIT_DECLARE_BIG_FOUR(ReVarValue);
	};

	class DisplayVar : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc& plan_next, int)->void;
		explicit DisplayVar(const std::string& name = "DisplayVar");
		virtual ~DisplayVar();
		CODEIT_REGISTER_TYPE(DisplayVar);
		CODEIT_DECLARE_BIG_FOUR(DisplayVar);
	};

	class ProgramBlock :public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc& plan_next, int)->void;
		//GetInfo() {	this->command().setName("get_i");}
		explicit ProgramBlock(const std::string& name = "programblock");
		virtual ~ProgramBlock();
		CODEIT_REGISTER_TYPE(ProgramBlock);
		CODEIT_DECLARE_BIG_FOUR(ProgramBlock);
	};

	class OpenRNG : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc& plan_next, int)->void;
		explicit OpenRNG(const std::string& name = "OpenRNG");
		virtual ~OpenRNG();
		CODEIT_REGISTER_TYPE(OpenRNG);
		CODEIT_DECLARE_BIG_FOUR(OpenRNG);
	};


	class SetDynamicEngine : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc& plan_next, int)->void;
		explicit SetDynamicEngine(const std::string& name = "SetDynamicEngine");
		virtual ~SetDynamicEngine();
		CODEIT_REGISTER_TYPE(SetDynamicEngine);
		CODEIT_DECLARE_BIG_FOUR(SetDynamicEngine);
	};

	class SetControlMode : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc& plan_next, int)->void;
		explicit SetControlMode(const std::string& name = "SetControlMode");
		virtual ~SetControlMode();
		CODEIT_REGISTER_TYPE(SetControlMode);
		CODEIT_DECLARE_BIG_FOUR(SetControlMode);
	};

	class SetCollisionDetection : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc& plan_next, int)->void;
		explicit SetCollisionDetection(const std::string& name = "SetCollisionDetection");
		virtual ~SetCollisionDetection();
		CODEIT_REGISTER_TYPE(SetCollisionDetection);
		CODEIT_DECLARE_BIG_FOUR(SetCollisionDetection);
	};

	class SetCheckMasterConnection : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc& plan_next, int)->void;
		explicit SetCheckMasterConnection(const std::string& name = "SetCheckMasterConnection");
		virtual ~SetCheckMasterConnection();
		CODEIT_REGISTER_TYPE(SetCheckMasterConnection);
		CODEIT_DECLARE_BIG_FOUR(SetCheckMasterConnection);
	};

	class CJointT : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc& plan_next, int)->void;
		explicit CJointT(const std::string& name = "CJointT");
		virtual ~CJointT();
		CODEIT_REGISTER_TYPE(CJointT);
		CODEIT_DECLARE_BIG_FOUR(CJointT);
	};

	class SetMaxToq : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~SetMaxToq();
		explicit SetMaxToq(const std::string& name = "SetMaxToq");
		CODEIT_REGISTER_TYPE(SetMaxToq);
		CODEIT_DECLARE_BIG_FOUR(SetMaxToq);
	};

	class SwitchToCST : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~SwitchToCST();
		explicit SwitchToCST(const std::string& name = "SwitchToCST");
		CODEIT_REGISTER_TYPE(SwitchToCST);
		CODEIT_DECLARE_BIG_FOUR(SwitchToCST);
	};

	class SwitchToCSP : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~SwitchToCSP();
		explicit SwitchToCSP(const std::string& name = "SwitchToCSP");
		CODEIT_REGISTER_TYPE(SwitchToCSP);
		CODEIT_DECLARE_BIG_FOUR(SwitchToCSP);
	};


	class SetJointZeros : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~SetJointZeros();
		explicit SetJointZeros(const std::string& name = "SetZeros");
		CODEIT_REGISTER_TYPE(SetJointZeros);
		CODEIT_DECLARE_BIG_FOUR(SetJointZeros);
	};

	class SetPayload:public BasisFunc{
	public:
		virtual void prepareNrt(BasisFunc&, int);
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~SetPayload();
		explicit SetPayload(const std::string& name = "SetPayload");
		CODEIT_REGISTER_TYPE(SetPayload);
		CODEIT_DECLARE_BIG_FOUR(SetPayload);
	};

	class SetTool :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~SetTool();
		explicit SetTool(const std::string& name = "SetTool");
		CODEIT_REGISTER_TYPE(SetTool);
		CODEIT_DECLARE_BIG_FOUR(SetTool);
	};

	class GetStatusWord :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~GetStatusWord();
		explicit GetStatusWord(const std::string& name = "GetStatusWord");
		CODEIT_REGISTER_TYPE(GetStatusWord);
		CODEIT_DECLARE_BIG_FOUR(GetStatusWord);
	};

	class GetErrorInfo :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~GetErrorInfo();
		explicit GetErrorInfo(const std::string& name = "GetErrorInfo");
		CODEIT_REGISTER_TYPE(GetErrorInfo);
		CODEIT_DECLARE_BIG_FOUR(GetErrorInfo);
	};

	class NotRunExecute :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~NotRunExecute();
		explicit NotRunExecute(const std::string& name = "NotRunExecute");
		CODEIT_REGISTER_TYPE(NotRunExecute);
		CODEIT_DECLARE_BIG_FOUR(NotRunExecute);
	};


	class SetGlobalSwitchFlag : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc& plan_next, int)->void;
		explicit SetGlobalSwitchFlag(const std::string& name = "SetGlobalSwitchFlag");
		virtual ~SetGlobalSwitchFlag();
		CODEIT_REGISTER_TYPE(SetGlobalSwitchFlag);
		CODEIT_DECLARE_BIG_FOUR(SetGlobalSwitchFlag);
	};

}



#endif
