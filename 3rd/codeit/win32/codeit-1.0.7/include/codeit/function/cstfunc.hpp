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
		//�ؽ�״̬����//
		vector<double> joint_pos, joint_vel, joint_toq;
		
		//�ѿ���״̬����//
		Wobj wobj; double wobj_pm[16]{ 0 };//��������ϵ
		Tool tool; double tool_pm[16]{ 0 };//��������ϵ
		double pm_now[16]{ 0 }, PqEnd[7] = { 0 };//������ʵ��ĩ����̬��ת�������Ԫ��
		double begin_pm[16]{ 0 };//������ʵ�ʳ�ʼĩ����̬��ת����
		double cart_act_vel[6] = { 0 };//������ĩ���ٶ�
		
		//�����˵ѿ�����ά���ʩ�ӵ�FT//
		double FT_in_world[6], FT_in_tool[6];//��������ϵ�빤��������
		double FT_in_world_init[6], FT_in_tool_init[6];//count()==1ʱ�ĳ�ֵ

		//�����������ò���//
		vector<LPF1st> filter; double cutoff_freq;//һ�׵�ͨ�˲�������ֹƵ��
		std::int32_t coordinate;//�������ο�������ϵ
		vector<int> force_direction;//���ط���
		vector<double>	force_target, force_gain, vel_limit;//����Ŀ��ֵ������͵��ɼ����ٶ�����
		double cart_vel[6] = { 0 };//���ؼ�����ٶ�
		double pq_fc[7] = { 0 };//���ؼ����λ�ˣ�����ڵ�ǰλ��pm_now��������
		double dx_pm[16] = { 0 };//���ؼ����λ�ˣ�����ڳ�ʼλ�ˣ�������


		//�ѿ�����������//
		double PqEnd0[7] = { 0 }; 
		vector<double> KPP, KPV, KIV;//λ�û��������棬�ٶȻ��������棬�ٶȻ���������
		vector<double> vt_limit, error_sum_limit;//λ�û���������ֵ���ٶȻ�������ֵ
		vector<double> error_sum;//�ٶȻ�����ֵ
		double ft[6] = { 0 };//ĩ����άft����ֵ
		vector<double> ft_limit;//ĩ����άft����ֵ����ֵ
		vector<double> tor_ref;//���㵽�ؽڵ����ز���ֵ
		vector<double> tor_limit;//���㵽�ؽ����ز���ֵ����ֵ

		//Ħ����ǰ����������//
		vector<double> cf_coef, vf_coef, zero_check;//����Ħ��������ϵ��
		vector<double> fric_feedforward;//����Ħ����ǰ������ֵ

		//ĩ�˸��ز���//
		Load load;
		double load_vec[10];
		double end_part_iv[10];
		double end_part_load_iv[10];

		//ϵͳʱ�����//
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
