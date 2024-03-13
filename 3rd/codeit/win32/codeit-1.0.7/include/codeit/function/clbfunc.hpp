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


	//4��-TCP�궨������4����㹤������ϵԭ��λ��
	class CalibT4P : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;
		explicit CalibT4P(const std::string& name = "CalibT4P");
		CODEIT_REGISTER_TYPE(CalibT4P);
		CODEIT_DECLARE_BIG_FOUR(CalibT4P);
	};


	//5��-TCP�궨������5����㹤������ϵԭ��λ��
	class CalibT5P : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;
		explicit CalibT5P(const std::string& name = "CalibT5P");
		CODEIT_REGISTER_TYPE(CalibT5P);
		CODEIT_DECLARE_BIG_FOUR(CalibT5P);
	};


	//6��-TCP�궨������6����㹤������ϵԭ��λ��
	class CalibT6P : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;
		explicit CalibT6P(const std::string& name = "CalibT6P");
		CODEIT_REGISTER_TYPE(CalibT6P);
		CODEIT_DECLARE_BIG_FOUR(CalibT6P);
	};


	//ȫ�۶���ѧ�궨�����ڵ�������
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

	//ȫ�۶���ѧ�궨�����ڵ�������,txt���룬����
	class CalibCurDynTxt : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;
		virtual void collectNrt(BasisFunc&, int index = 0);

		explicit CalibCurDynTxt(const std::string& name = "CalibCurDynTxt");
		CODEIT_REGISTER_TYPE(CalibCurDynTxt);
		CODEIT_DECLARE_BIG_FOUR(CalibCurDynTxt);
	};


	//���ض���ѧ�궨�����ڵ�������
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


	//ĩ�˸��ر궨������ĩ����������
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

	//ĩ�˸��ر궨������ĩ����������
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