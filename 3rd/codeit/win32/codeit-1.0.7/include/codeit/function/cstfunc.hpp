#ifndef CODEIT_CST_FUNCTION_H_
#define CODEIT_CST_FUNCTION_H_
#include<codeit/function/basefunc.hpp>

namespace codeit::system { class ControlSystem; }
namespace codeit::function {

	class DragInCST : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~DragInCST();
		explicit DragInCST(const std::string& name = "DragInCST");
		CODEIT_REGISTER_TYPE(DragInCST);
		CODEIT_DECLARE_BIG_FOUR(DragInCST);
	};

	class SetToqInCST : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~SetToqInCST();
		explicit SetToqInCST(const std::string& name = "SetToqInCST");
		CODEIT_REGISTER_TYPE(SetToqInCST);
		CODEIT_DECLARE_BIG_FOUR(SetToqInCST);
	};

	class MoveJSInCST : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~MoveJSInCST();
		explicit MoveJSInCST(const std::string& name = "MoveJSInCST");
		CODEIT_REGISTER_TYPE(MoveJSInCST);
		CODEIT_DECLARE_BIG_FOUR(MoveJSInCST);
	};


	class MoveCCSInCST : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~MoveCCSInCST();
		explicit MoveCCSInCST(const std::string& name = "MoveCCSInCST");
		CODEIT_REGISTER_TYPE(MoveCCSInCST);
		CODEIT_DECLARE_BIG_FOUR(MoveCCSInCST);
	};


	class ForceModeInCST : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~ForceModeInCST();
		explicit ForceModeInCST(const std::string& name = "ForceModeInCST");
		CODEIT_REGISTER_TYPE(ForceModeInCST);
		CODEIT_DECLARE_BIG_FOUR(ForceModeInCST);
	};

	class ForceModeInCST2 : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~ForceModeInCST2();
		explicit ForceModeInCST2(const std::string& name = "ForceModeInCST2");
		CODEIT_REGISTER_TYPE(ForceModeInCST2);
		CODEIT_DECLARE_BIG_FOUR(ForceModeInCST2);
	};



	struct FCParam
	{
		//关节状态参数//
		vector<double> joint_pos, joint_vel, joint_toq;
		
		//笛卡尔状态参数//
		Wobj wobj; double wobj_pm[16]{ 0 };//世界坐标系
		Tool tool; double tool_pm[16]{ 0 };//工具坐标系
		double pm_now[16]{ 0 }, PqEnd[7] = { 0 };//机器人实际末端姿态旋转矩阵和四元数
		double begin_pm[16]{ 0 };//机器人实际初始末端姿态旋转矩阵
		double cart_act_vel[6] = { 0 };//机器人末端速度
		
		//机器人笛卡尔六维外界施加的FT//
		double FT_in_world[6], FT_in_tool[6];//世界坐标系与工作坐标下
		double FT_in_world_init[6], FT_in_tool_init[6];//count()==1时的初值

		//导纳力控设置参数//
		vector<LPF1st> filter; double cutoff_freq;//一阶低通滤波器及截止频率
		std::int32_t coordinate;//力控所参考的坐标系
		vector<int> force_direction;//力控方向
		vector<double>	force_target, force_gain, vel_limit;//力控目标值、增益和导纳计算速度限制
		double cart_vel[6] = { 0 };//力控计算的速度
		double pq_fc[7] = { 0 };//力控计算的位姿（相对于当前位姿pm_now）的增量
		double dx_pm[16] = { 0 };//力控计算的位姿（相对于初始位姿）的增量


		//笛卡尔三环参数//
		double PqEnd0[7] = { 0 }; 
		vector<double> KPP, KPV, KIV;//位置环比例增益，速度环比例增益，速度环积分增益
		vector<double> vt_limit, error_sum_limit;//位置环计算结果阈值，速度环积分阈值
		vector<double> error_sum;//速度环积分值
		double ft[6] = { 0 };//末端六维ft计算值
		vector<double> ft_limit;//末端六维ft计算值的阈值
		vector<double> tor_ref;//折算到关节的力矩补偿值
		vector<double> tor_limit;//折算到关节力矩补偿值的阈值

		//摩擦力前馈补偿参数//
		vector<double> cf_coef, vf_coef, zero_check;//各轴摩擦力补偿系数
		vector<double> fric_feedforward;//各轴摩擦力前馈力矩值

		//末端负载参数//
		Load load;
		double load_vec[10];
		double end_part_iv[10];
		double end_part_load_iv[10];

		//系统时间参数//
		double dt, ut, utBreak;
	};

	void updGlobalLoad(BasisFunc& plan, FCParam& param);
	std::int32_t getExternalFT(BasisFunc& plan, FCParam& param);
	void admittanceControl(BasisFunc& plan, FCParam& param);
	void cartesianPID(BasisFunc& plan, FCParam& param);
	void cartesianFTtoJointToq(BasisFunc& plan, FCParam& param);

	class FCStatic : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~FCStatic();
		explicit FCStatic(const std::string& name = "FCStatic");
		CODEIT_REGISTER_TYPE(FCStatic);
		CODEIT_DECLARE_BIG_FOUR(FCStatic);
	};

}




#endif
