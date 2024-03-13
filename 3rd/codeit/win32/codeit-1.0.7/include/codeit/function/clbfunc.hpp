#ifndef CODEIT_CALIBRATOR_FUNCTION_H_
#define CODEIT_CALIBRATOR_FUNCTION_H_
#include<codeit/function/basefunc.hpp>

namespace codeit::system { class ControlSystem; }
namespace codeit::function {

	class CalibJointZero : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;
		explicit CalibJointZero(const std::string& name = "CalibJointZero");
		CODEIT_REGISTER_TYPE(CalibJointZero);
		CODEIT_DECLARE_BIG_FOUR(CalibJointZero);
	};


	//4点-TCP标定，基于4点计算工具坐标系原点位置
	class CalibT4P : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;
		explicit CalibT4P(const std::string& name = "CalibT4P");
		CODEIT_REGISTER_TYPE(CalibT4P);
		CODEIT_DECLARE_BIG_FOUR(CalibT4P);
	};


	//5点-TCP标定，基于5点计算工具坐标系原点位置
	class CalibT5P : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;
		explicit CalibT5P(const std::string& name = "CalibT5P");
		CODEIT_REGISTER_TYPE(CalibT5P);
		CODEIT_DECLARE_BIG_FOUR(CalibT5P);
	};


	//6点-TCP标定，基于6点计算工具坐标系原点位置
	class CalibT6P : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;
		explicit CalibT6P(const std::string& name = "CalibT6P");
		CODEIT_REGISTER_TYPE(CalibT6P);
		CODEIT_DECLARE_BIG_FOUR(CalibT6P);
	};


	//全臂动力学标定，基于电流反馈
	class CalibCurDyn : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;
		auto virtual executeRT(BasisFunc&, int index)->int;
		virtual void collectNrt(BasisFunc&, int index = 0);

		explicit CalibCurDyn(const std::string& name = "CalibCurDyn");
		CODEIT_REGISTER_TYPE(CalibCurDyn);
		CODEIT_DECLARE_BIG_FOUR(CalibCurDyn);
	};

	//全臂动力学标定，基于电流反馈,txt导入，仿真
	class CalibCurDynTxt : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;
		virtual void collectNrt(BasisFunc&, int index = 0);

		explicit CalibCurDynTxt(const std::string& name = "CalibCurDynTxt");
		CODEIT_REGISTER_TYPE(CalibCurDynTxt);
		CODEIT_DECLARE_BIG_FOUR(CalibCurDynTxt);
	};


	//负载动力学标定，基于电流反馈
	class CalibCurLoad : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;
		auto virtual executeRT(BasisFunc&, int index)->int;
		virtual void collectNrt(BasisFunc&, int index = 0);

		explicit CalibCurLoad(const std::string& name = "CalibCurLoad");
		CODEIT_REGISTER_TYPE(CalibCurLoad);
		CODEIT_DECLARE_BIG_FOUR(CalibCurLoad);
	};


	//末端负载标定，基于末端力传感器
	class CalibFtDyn : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;
		auto virtual executeRT(BasisFunc&, int index)->int;
		virtual void collectNrt(BasisFunc&, int index = 0);

		explicit CalibFtDyn(const std::string& name = "CalibFtDyn");
		CODEIT_REGISTER_TYPE(CalibFtDyn);
		CODEIT_DECLARE_BIG_FOUR(CalibFtDyn);
	};

	//末端负载标定，基于末端力传感器
	class CalibFtDynTxt : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;
		auto virtual executeRT(BasisFunc&, int index)->int;
		virtual void collectNrt(BasisFunc&, int index = 0);

		explicit CalibFtDynTxt(const std::string& name = "CalibFtDynTxt");
		CODEIT_REGISTER_TYPE(CalibFtDynTxt);
		CODEIT_DECLARE_BIG_FOUR(CalibFtDynTxt);
	};

}





#endif