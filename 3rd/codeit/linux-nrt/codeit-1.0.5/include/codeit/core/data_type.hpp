#ifndef CODEIT_DATA_TYPE_H_
#define CODEIT_DATA_TYPE_H_
#include <codeit/core/basictype.hpp>
namespace codeit::core
{
	class JointTarget
	{
	public:
		double joint_vec[MAX_DOFS] = { 0 };
	};
	
	class ZeroComp
	{
	public:
		double zero_comp[MAX_DOFS] = { 0 };
	};
	
	class Pos
	{
	public:
		double x, y, z;
	};
	
	class Orient
	{
	public:
		double q1, q2, q3, q4;
	};
	
	class Pose
	{
	public:
		Pos trans;
		Orient rot;
	};
	
	class RobotTarget
	{
	public:
		Pos trans;
		Orient rot;
	};
	
	class Zone
	{
	public:
		double dis;	//转弯区长度m
		double per;	//转弯区百分比分数表达
	};
	
	class Speed
	{
	public:
		double w_per;	//关节速度百分比
		double v_tcp;	//TCP线速度mm/s
		double w_tcp;	//空间旋转速度°/s
		double w_ext;	//外部轴角速度°/s
		double v_ext;	//外部轴线速度mm/s
	};
	
	class Load
	{
	public:
		double mass;
		Pos cog;
		double tensor[6];
	};

	class Tool
	{
	public:
		Pose pose;
	};

	class Wobj
	{
	public:
		Pose pose;
	};
	

	class TeachTarget {
	public:
		JointTarget jointTarget;
		Tool tool;
		Wobj wobj;
		RobotTarget robotTarget;
	};
	
	class DataTypeSize {
	public:
		static Size jointTarget;
		static Size zeroComp;
		static Size pos;
		static Size orient;
		static Size pose;
		static Size robotTarget;
		static Size zone;
		static Size speed;
		static Size load;
		static Size tool;
		static Size wobj;
		static Size teachTarget;
	};

	auto s_jointtarget2vec(const JointTarget& jointTarget, double* vec)->double*;
	auto s_vec2jointtarget(const double* vec, JointTarget& jointTarget)->void;

	auto s_zerocomp2vec(const ZeroComp& zeroComp, double* vec)->double*;
	auto s_vec2zerocomp(const double* vec, ZeroComp& zeroComp)->void;

	auto s_zone2vec(const Zone& zone, double* vec)->double*;
	auto s_vec2zone(const double* vec, Zone& zone)->void;

	auto s_speed2vec(const Speed& speed, double* vec)->double*;
	auto s_vec2speed(const double* vec, Speed& speed)->void;

	auto s_pos2vec(const Pos& pos, double* vec)->double*;
	auto s_vec2pos(const double* vec,Pos& pos)->void;

	auto s_orient2vec(const Orient& rot, double* vec)->double*;
	auto s_vec2orient(const double* vec, Orient& rot)->void;

	auto s_pose2vec(const Pose& pose, double* vec)->double*;
	auto s_vec2pose(const double* vec, Pose& pose)->void;

	auto s_load2vec(const Load& load, double* vec)->double*;
	auto s_vec2load(const double* vec, Load& load)->void;

	auto s_tool2vec(const Tool& tool, double* vec)->double*;
	auto s_vec2tool(const double* vec, Tool& tool)->void;

	auto s_wobj2vec(const Wobj& wobj, double* vec)->double*;
	auto s_vec2wobj(const double* vec, Wobj& wobj)->void;

	auto s_robtarget2vec(const RobotTarget& target, double* vec)->double*;
	auto s_vec2robtarget(const double* vec, RobotTarget& target)->void;
}

#endif
