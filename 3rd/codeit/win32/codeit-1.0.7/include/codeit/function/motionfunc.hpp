#ifndef CODEIT_MOTION_FUNCTION_H_
#define CODEIT_MOTION_FUNCTION_H_
#include<codeit/function/basefunc.hpp>

namespace codeit::system { class ControlSystem; }
namespace codeit::function {
#define CHECK_SIGULARITY \
		std::int32_t sr = model()->sigularArbiterPool().at(0).isSingular(*model());\
		if (sr < 0)\
			return sr;


	struct SetInputMovement;
	struct SetActiveMotor;
	auto set_input_movement(const std::map<std::string_view, std::string_view>& cmd_params, BasisFunc& plan, SetInputMovement& param)->void;
	auto check_input_movement(const std::map<std::string_view, std::string_view>& cmd_params, BasisFunc& plan, SetInputMovement& param, SetActiveMotor& active)->void;
	auto set_active_motor(const std::map<std::string_view, std::string_view>& cmd_params, BasisFunc& plan, SetActiveMotor& param)->void;
	
	class Idle :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);
		virtual int executeRT(BasisFunc&, int);
		virtual void collectNrt(BasisFunc&, int);
		virtual ~Idle();
		explicit Idle(const std::string& name = "Idle");
		CODEIT_REGISTER_TYPE(Idle);
		CODEIT_DECLARE_BIG_FOUR(Idle);
	};


	class Enable :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);
		virtual int executeRT(BasisFunc&, int);
		virtual void collectNrt(BasisFunc&, int);
		virtual ~Enable();
		explicit Enable(const std::string& name = "Enable");
		CODEIT_REGISTER_TYPE(Enable);
		CODEIT_DECLARE_BIG_FOUR(Enable);
	};

	class Disable : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		virtual ~Disable();
		explicit Disable(const std::string& name = "Disable");
		CODEIT_REGISTER_TYPE(Disable);
		CODEIT_DECLARE_BIG_FOUR(Disable);
	};

	class Mode : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		virtual ~Mode();
		explicit Mode(const std::string& name = "Mode");
		CODEIT_REGISTER_TYPE(Mode);
		CODEIT_DECLARE_BIG_FOUR(Mode);
	};

	class Home : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		virtual ~Home();
		explicit Home(const std::string& name = "Home");
		CODEIT_REGISTER_TYPE(Home);
		CODEIT_DECLARE_BIG_FOUR(Home);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};

	class Reset : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;

		virtual ~Reset();
		explicit Reset(const std::string& name = "Reset");
		CODEIT_REGISTER_TYPE(Reset);
		CODEIT_DECLARE_BIG_FOUR(Reset);
	};

	class Recover : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~Recover();
		explicit Recover(const std::string& name = "Recover");
		CODEIT_REGISTER_TYPE(Recover);
		CODEIT_DECLARE_BIG_FOUR(Recover);
	};

	class Show :public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		virtual ~Show();
		explicit Show(const std::string& name = "Show");
		CODEIT_REGISTER_TYPE(Show);
		CODEIT_DECLARE_BIG_FOUR(Show);
	};


	struct SetActiveMotor { std::vector<int> active_motor; };
	struct SetInputMovement
	{
		std::vector<double> axis_begin_pos_vec;
		std::vector<double> axis_pos_vec;
		std::vector<double> axis_vel_vec;
		std::vector<double> axis_acc_vec;
		std::vector<double> axis_dec_vec;
		std::vector<double> axis_jerk_vec;
	};

	struct LookAheadParam
	{
		vector<double> Tt;//Tt为本指令S规划出的时间特征参数//
		double tool_pm[16]{ 0 }, wobj_pm[16]{ 0 };
		double ee_end_pm[16];
		// 下一条指令规划的轴空间参数 //
		vector<double> dpos_joint_next, dvel_joint_next, dacc_joint_next, djerk_joint_next, t_next, Tt_next;//Tt为S规划出的时间特征参数//
		bool zone_enabled = false;
		Zone zone;
	};

#define ZONE_INIT\
	if (cmd_param.first == "zone")\
	{\
		auto iter = model()->variablePool().findByName(std::string(cmdParams().at(cmd_param.first)));\
		if (iter == model()->variablePool().end()) THROW_FILE_LINE("param not exist:" + std::string(cmdParams().at(cmd_param.first)));\
		auto s = dynamic_cast<codeit::model::MatrixVariable*>(&*iter);\
		if(s->data().size()!=DataTypeSize::zone)THROW_FILE_LINE("dimension mismatch:" + std::string(cmdParams().at(cmd_param.first)));\
		s_vec2zone(s->data().data(), param.zone);\
		param.zone_enabled = 1;\
		if (abs(param.zone.per < 1.0e-5))\
			param.zone_enabled = 0;\
		if (param.zone.per > 0.5)\
				THROW_FILE_LINE("input beyond range: zone");\
	}\
	else if (cmd_param.first == "zone_var")\
	{\
		if (cmd_param.second == "fine")\
		{\
			param.zone.dis = 0.0;\
			param.zone.per = 0.0;\
			param.zone_enabled = 0;\
		}\
		else\
		{\
			s_vec2zone(std::any_cast<Matrix>(cal.calculateExpression("zone(" + std::string(cmdParams().at(cmd_param.first)) + ")").second).data(), param.zone);\
			param.zone_enabled = 1;\
			if (abs(param.zone.per < 1.0e-5))\
				param.zone_enabled = 0;\
			if (param.zone.per > 0.5)\
				THROW_FILE_LINE("input beyond range: zone");\
		}\
	}\
	
#define SPEED_INIT\
	if (cmd_param.first == "speed")\
	{\
		auto iter = model()->variablePool().findByName(std::string(cmdParams().at(cmd_param.first)));\
		if (iter == model()->variablePool().end()) THROW_FILE_LINE("param not exist:" + std::string(cmdParams().at(cmd_param.first)));\
		auto s = dynamic_cast<codeit::model::MatrixVariable*>(&*iter);\
		if (s->data().size() != DataTypeSize::speed) THROW_FILE_LINE("dimension mismatch:" + std::string(cmdParams().at(cmd_param.first)));\
		s_vec2speed(s->data().data(), param.speed);\
	}\
	else if (cmd_param.first == "speed_var")\
	{\
		s_vec2speed(std::any_cast<Matrix>(cal.calculateExpression("speed(" + std::string(cmdParams().at(cmd_param.first)) + ")").second).data(), param.speed);\
	}\

#define LOAD_INIT\
	if (cmd_param.first == "load")\
	{\
		auto iter = model()->variablePool().findByName(std::string(cmdParams().at(cmd_param.first)));\
		if (iter == model()->variablePool().end()) THROW_FILE_LINE("param not exist:" + std::string(cmdParams().at(cmd_param.first)));\
		auto s = dynamic_cast<codeit::model::MatrixVariable*>(&*iter);\
		if(s->data().size()!=DataTypeSize::load)THROW_FILE_LINE("dimension mismatch:" + std::string(cmdParams().at(cmd_param.first)));\
		s_vec2load(s->data().data(), param.load);\
	}\
	else if (cmd_param.first == "load_var")\
	{\
		s_vec2load(std::any_cast<Matrix>(cal.calculateExpression("load(" + std::string(cmdParams().at(cmd_param.first)) + ")").second).data(), param.load);\
	}\

#define JOINTTARGET_INIT\
	if (cmd_param.first == "jointtarget")\
	{\
		auto iter0 = model()->pointPool().findByName(std::string(cmdParams().at(cmd_param.first)));\
		if (iter0 == model()->pointPool().end()) THROW_FILE_LINE("param not exist:" + std::string(cmdParams().at(cmd_param.first)));\
		param.jointTarget=iter0->jointTarget();\
	}\
	else if (cmd_param.first == "jointtarget_var")\
	{\
		s_vec2jointtarget(std::any_cast<Matrix>(cal.calculateExpression("jointtarget(" + std::string(cmd_param.second) + ")").second).data(), param.jointTarget);\
	}\

#define ROBOTTARGET_INIT\
	if (cmd_param.first == "robottarget")\
	{\
		auto iter = model()->pointPool().findByName(std::string(cmdParams().at(cmd_param.first)));\
		if (iter == model()->pointPool().end()) THROW_FILE_LINE("param not exist:" + std::string(cmdParams().at(cmd_param.first)));\
		param.robotTarget=iter->robotTarget();\
		param.wobj = iter->wobj();\
		param.tool = iter->tool();\
	}\
	else if (cmd_param.first == "robottarget_var")\
	{\
		s_vec2robtarget(std::any_cast<Matrix>(cal.calculateExpression("robottarget(" + std::string(cmd_param.second) + ")").second).data(), param.robotTarget);\
	}\

#define WOBJ_INIT\
	if (cmd_param.first == "wobj")\
	{\
		auto iter = model()->wobjPool().findByName(std::string(cmdParams().at(cmd_param.first)));\
		if (iter == model()->wobjPool().end()) THROW_FILE_LINE("param not exist:" + std::string(cmdParams().at(cmd_param.first)));\
		param.wobj = iter->wobj();\
	}\
	else if (cmd_param.first == "wobj_var")\
	{\
		s_vec2wobj(std::any_cast<Matrix>(cal.calculateExpression("wobj(" + std::string(cmdParams().at(cmd_param.first)) + ")").second).data(), param.wobj);\
	}\

#define TOOL_INIT\
	if (cmd_param.first == "tool")\
	{\
		auto iter = model()->toolPool().findByName(std::string(cmdParams().at(cmd_param.first)));\
		if (iter == model()->toolPool().end()) THROW_FILE_LINE("param not exist:" + std::string(cmdParams().at(cmd_param.first)));\
		param.tool = iter->tool();\
	}\
	else if (cmd_param.first == "tool_var")\
	{\
		s_vec2tool(std::any_cast<Matrix>(cal.calculateExpression("tool(" + std::string(cmdParams().at(cmd_param.first)) + ")").second).data(), param.tool);\
	}\

	struct ExternMoveJParam
	{
		std::vector<double> extern_begin_pos_vec;
		std::vector<double> extern_pos_vec;
		std::vector<double> extern_vel_vec;
		std::vector<double> extern_acc_vec;
		std::vector<double> extern_dec_vec;
		std::vector<double> extern_jerk_vec;

		///*** Extern Motor ***///
		vector<double> angular_distance, ForwardOrReverse;
		vector<double> ratio;//减速度÷加速度
		vector<double> vlim, alim;
		vector<int> init_flag;
		vector<int> running_flag;

		// pos表示的是相对于初始位置的增量 //
		vector<double> dpos_joint, dvel_joint, dacc_joint, djerk_joint, t, Tt;//Tt为S规划出的时间特征参数//
		vector<double> dpos_joint_raw, dvel_joint_raw, dacc_joint_raw, djerk_joint_raw;
		// 下一条指令规划的轴空间参数 //
		vector<double> dpos_joint_next, dvel_joint_next, dacc_joint_next, djerk_joint_next, t_next, Tt_next;//Tt为S规划出的时间特征参数//

		bool zone_enabled = false;
		Load load;
		double tool_pm[16]{ 0 }, wobj_pm[16]{ 0 };
		codeit::model::JointPlanner* jointPlanner;
	};

	struct MoveAbsJParam : public SetActiveMotor, SetInputMovement, LookAheadParam
	{
		vector<double> angular_distance, ForwardOrReverse;
		vector<double> vlim, alim;
		vector<int> init_flag;
		vector<int> running_flag;

		// pos表示的是相对于初始位置的增量 //
		vector<double> dpos_joint, dvel_joint, dacc_joint, djerk_joint, t;
		vector<double> dpos_joint_raw, dvel_joint_raw, dacc_joint_raw, djerk_joint_raw;
		
		std::int32_t algorithm_error_code{ 0 };
		char lgorithm_error_msg[1024]{};
		
		Load load;
		Speed speed;
		JointTarget jointTarget;
		Wobj wobj;
		Tool tool;
		codeit::model::JointPlanner* jointPlanner;
	};
	struct MoveJParam : public SetActiveMotor, SetInputMovement, LookAheadParam
	{
		vector<double> angular_distance, ForwardOrReverse;
		vector<double> vlim, alim;
		vector<int> init_flag;
		vector<int> running_flag;

		// pos表示的是相对于初始位置的增量 //
		vector<double> dpos_joint, dvel_joint, dacc_joint, djerk_joint, t;
		vector<double> dpos_joint_raw, dvel_joint_raw, dacc_joint_raw, djerk_joint_raw;

		std::int32_t algorithm_error_code{ 0 };
		char lgorithm_error_msg[1024]{};

		Load load;
		Speed speed;
		RobotTarget robotTarget;
		Wobj wobj;
		Tool tool;
		codeit::model::JointPlanner* jointPlanner;
	};
	struct MoveLParam : public SetInputMovement, LookAheadParam
	{
		// 初始位置 //
		double begin_pq[7] = { 0 }, begin_pm[16] = { 0 };
		// 目标位置 //
		std::vector<double> ee_end_pq;
		double trans_acc, trans_vel, trans_dec, trans_jerk;
		double angular_acc, angular_vel, angular_dec, angular_jerk;
		// 实时数据 //
		double st[2] = { 0,0 }, vt[2] = { 0,0 }, at[2] = { 0,0 }, jt[2] = { 0,0 };
		double pm[16];
		double t[2] = { 0 };

		vector<double> pos_joint, vel_joint, acc_joint, jerk_joint;// MoveL下的当前关节指令值 //
		vector<double> dpos_joint, dvel_joint, dacc_joint, djerk_joint;// MoveL下的当前关节指令值增加值 //

		std::int32_t algorithm_error_code{ 0 };
		char lgorithm_error_msg[1024]{};
		bool zone_enabled = false;
		Zone zone;
		Speed speed;
		Load load;
		Wobj wobj;
		Tool tool;
		RobotTarget robotTarget;
		codeit::model::MoveLPlanner* moveLPlanner;
	};
	struct MoveCParam : public SetInputMovement, LookAheadParam
	{
		// 初始位置 //
		double begin_pq[7], begin_pm[16];
		// 目标位置 //
		double ee_mid_pq[7], ee_end_pq[7];
		double ee_mid_pm[16];
		// 圆的参数 //
		double C[3], R, theta, ori_theta, axis[3];
		bool circle_flag;

		// 规划参数 //
		double trans_acc, trans_vel, trans_dec, trans_jerk;
		double angular_acc, angular_vel, angular_dec, angular_jerk;
		// 实时数据 //
		double st[2] = { 0,0 }, vt[2] = { 0,0 }, at[2] = { 0,0 }, jt[2] = { 0,0 };
		double pm[16];
		double t[2] = { 0 };

		vector<double> pos_joint, vel_joint, acc_joint, jerk_joint;// MoveL下的当前关节指令值 //
		vector<double> dpos_joint, dvel_joint, dacc_joint, djerk_joint;// MoveL下的当前关节指令值增加值 //

		std::int32_t algorithm_error_code{ 0 };
		char lgorithm_error_msg[1024]{};
		bool zone_enabled = false;
		Zone zone;
		Speed speed;
		Load load;
		Wobj wobj;
		Tool tool;
		RobotTarget robotTarget;
		RobotTarget midRobotTarget;
		codeit::model::MoveCPlanner* moveCPlanner;
	};
	struct MoveSParam : public SetInputMovement
	{
		// 初始位置 //
		double begin_pq[7], begin_pm[16];
		// 中间位置集 //
		Size ptsNum;
		vector<double> ee_set_pq;
		vector<double> norm_ori;
		vector<double> u;
		vector<double> P_vec;
		vector<double> quatP_vec;
		vector<vector<double> > teachPts;//将位置集转成二维矩阵，每一列为一个位置
		// 终点位置 //
		std::vector<double> ee_end_pq;
		double ee_end_pm[16];
		// 样条曲线的参数 //
		int p = 3;//三次样条拟合
		double arcLength = 0;

		// 规划参数 //
		double trans_acc, trans_vel, trans_dec, trans_jerk;
		double angular_acc, angular_vel, angular_dec, angular_jerk;
		double Tt[4];
		// 实时数据 //
		double uSpline = 0;
		std::vector<double> ds0, dds0;
		double st{ 0 }, vt{ 0 }, at{ 0 }, jt{ 0 };
		double pm[16];
		double t{ 0 };

		vector<double> pos_joint, vel_joint, acc_joint, jerk_joint;// MoveL下的当前关节指令值 //
		vector<double> dpos_joint, dvel_joint, dacc_joint, djerk_joint;// MoveL下的当前关节指令值增加值 //


		// 下一条指令规划的轴空间参数 //
		vector<double> dpos_joint_next, dvel_joint_next, dacc_joint_next, djerk_joint_next, t_next, Tt_next;//Tt为S规划出的时间特征参数//

		std::int32_t algorithm_error_code{ 0 };
		char lgorithm_error_msg[1024]{};
		double tool_pm[16], wobj_pm[16];
		
		bool zone_enabled = false;
		Zone zone;
		Speed speed;
		Load load;
		Wobj wobj;
		Tool tool;
		RobotTarget robotTarget;
		codeit::model::MoveSPlanner* moveSPlanner;

		bool moveL_phase{ true };
		MoveLParam mvl_param;
	};
	struct MoveLLParam
	{
		std::vector<double> axis_begin_pos_vec;
		// 初始位置 //
		double begin_pq[7], begin_pm[16];
		// 目标位置 //
		double ee_end_pq[7], ee_end_pm[16];

		// 针对Bezier曲线 //
		vector<vector<double> > teachPts;//将位置集转成二维矩阵，每一列为一个位置
		vector<vector<double> > q_line0, q_line;
		int p = 3;
		vector<vector<double> > t0;
		vector<vector<double> > n0;
		vector<vector<double> > t5;
		vector<vector<double> > n5;
		vector<vector<double> > P;
		vector<double> uk, lamda;
		double uBezier = 0;
		std::vector<double> b, db0, ddb0;
		double arcLength = 0;
		vector<double> quatP_vec;//姿态向量

		bool moveLLContinue = true;

		double trans_acc, trans_vel, trans_dec, trans_jmax, trans_jmin;
		double angular_acc, angular_vel, angular_dec, angular_jmax, angular_jmin;
		double Tt[4 * 2];
		// 实时数据 //
		double st{ 0 }, vt{ 0 }, at{ 0 }, jt{ 0 };
		double pm[16];
		double t{ 0 };

		vector<double> pos_joint, vel_joint, acc_joint, jerk_joint;// MoveLL下的当前关节指令值 //
		vector<double> dpos_joint, dvel_joint, dacc_joint, djerk_joint;// MoveLL下的当前关节指令值增加值 //
		// 下一条指令规划的轴空间参数 //
		vector<double> dpos_joint_next, dvel_joint_next, dacc_joint_next, djerk_joint_next, t_next, Tt_next;//Tt为S规划出的时间特征参数//

		std::int32_t algorithm_error_code{ 0 };
		char lgorithm_error_msg[1024]{};
		double tool_pm[16], wobj_pm[16];
		bool zone_enabled = false;
		Zone zone;
		Speed speed;
		Load load;
		Wobj wobj;
		Tool tool;
		RobotTarget robotTarget;
		codeit::model::MoveLLPlanner* moveLLPlanner;

		MoveLParam mvl_param;
	};

	struct ServoSeriesParam
	{
		vector<std::string> joint_set;
		std::vector<double> time, time_whole;
		double scale, t, tt, ratio, stop_time;
		vector<std::vector<double>> joint_mat;
		vector<std::vector<double>> pos;
		vector<std::vector<double>> pos_p1, pos_p2, pos_p3;
		JointTarget tmp;
		std::vector<double> axis_pos_vec,dpos_joint;
	};


	class MoveAbsJ :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);
		virtual int executeRT(BasisFunc&, int);
		virtual void collectNrt(BasisFunc&, int);
		virtual ~MoveAbsJ();
		explicit MoveAbsJ(const std::string& name = "MoveAbsJ");
		CODEIT_REGISTER_TYPE(MoveAbsJ);
		CODEIT_DECLARE_BIG_FOUR(MoveAbsJ);
	};

	class MoveJ :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);
		virtual int executeRT(BasisFunc&, int);
		virtual void collectNrt(BasisFunc&, int);
		virtual ~MoveJ();
		explicit MoveJ(const std::string& name = "MoveJ");
		CODEIT_REGISTER_TYPE(MoveJ);
		CODEIT_DECLARE_BIG_FOUR(MoveJ);
	};

	class MoveL :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);
		virtual int executeRT(BasisFunc&, int);
		virtual void collectNrt(BasisFunc&, int);
		virtual ~MoveL();
		explicit MoveL(const std::string& name = "MoveL");
		CODEIT_REGISTER_TYPE(MoveL);
		CODEIT_DECLARE_BIG_FOUR(MoveL);
	};

	class MoveC :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);
		virtual int executeRT(BasisFunc&, int);
		virtual void collectNrt(BasisFunc&, int);
		virtual ~MoveC();
		explicit MoveC(const std::string& name = "MoveC");
		CODEIT_REGISTER_TYPE(MoveC);
		CODEIT_DECLARE_BIG_FOUR(MoveC);
	};

	class MoveS :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);
		virtual int executeRT(BasisFunc&, int);
		virtual void collectNrt(BasisFunc&, int);
		virtual ~MoveS();
		explicit MoveS(const std::string& name = "MoveS");
		CODEIT_REGISTER_TYPE(MoveS);
		CODEIT_DECLARE_BIG_FOUR(MoveS);
	};

	class MoveLL :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);
		virtual int executeRTMoveLL(BasisFunc&, int, double* moveLL_pq_set = nullptr, Size moveLL_num = 0, bool moveLL_continue = false, bool first_moveLL = false);
		virtual void collectNrt(BasisFunc&, int);
		virtual ~MoveLL();
		explicit MoveLL(const std::string& name = "MoveLL");
		CODEIT_REGISTER_TYPE(MoveLL);
		CODEIT_DECLARE_BIG_FOUR(MoveLL);
	};

	class JogJ : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~JogJ();
		explicit JogJ(const std::string& name = "JogJ_plan");
		CODEIT_REGISTER_TYPE(JogJ);
		CODEIT_DECLARE_BIG_FOUR(JogJ);

	};

	class JogC : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~JogC();
		explicit JogC(const std::string& name = "JogC_plan");
		CODEIT_REGISTER_TYPE(JogC);
		CODEIT_DECLARE_BIG_FOUR(JogC);

	};

	class ServoJ : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~ServoJ();
		explicit ServoJ(const std::string& name = "ServoJ_plan");
		CODEIT_REGISTER_TYPE(ServoJ);
		CODEIT_DECLARE_BIG_FOUR(ServoJ);

	};

	class ServoSine : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;
		auto virtual executeRT(BasisFunc&, int index)->int;
		virtual void collectNrt(BasisFunc&, int index = 0);

		explicit ServoSine(const std::string& name = "ServoSine");
		CODEIT_REGISTER_TYPE(ServoSine);
		CODEIT_DECLARE_BIG_FOUR(ServoSine);
	};

	class ServoSeries : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;
		auto virtual executeRT(BasisFunc&, int index)->int;
		virtual void collectNrt(BasisFunc&, int index = 0);

		explicit ServoSeries(const std::string& name = "ServoSeries");
		CODEIT_REGISTER_TYPE(ServoSeries);
		CODEIT_DECLARE_BIG_FOUR(ServoSeries);
	};


	class ServoSeriesDynamic : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;
		auto virtual executeRT(BasisFunc&, int index)->int;
		virtual void collectNrt(BasisFunc&, int index = 0);

		explicit ServoSeriesDynamic(const std::string& name = "ServoSeriesDynamic");
		CODEIT_REGISTER_TYPE(ServoSeriesDynamic);
		CODEIT_DECLARE_BIG_FOUR(ServoSeriesDynamic);
	};

}
#endif