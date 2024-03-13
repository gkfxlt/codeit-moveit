#ifndef CODEIT_BASE_FUNCTION_H_
#define CODEIT_BASE_FUNCTION_H_
#include<codeit/core/basictype.hpp>
#include<codeit/core/core.hpp>
#include<codeit/controller/controller.hpp>
#include<codeit/model/allmodel.hpp>
#include<codeit/system/errorinfo.hpp>
#include<codeit/core/data_type.hpp>
#include<codeit/sensor/sensor.hpp>
#include <charconv>
#define COUT_PLAN(p) LOG_COUT << "    " << p->cmdId() << "---"
#define MOUT_PLAN(p) p->controller()->mout() << "RT  " << p->cmdId() << "---"
using namespace codeit::core;
namespace codeit::system { class ControlSystem; }

namespace codeit::function {
	
	auto common_plan_options(BasisFunc& func)->void;
	auto split(const std::string& tmp, std::vector<std::string>& result)->void;
	auto check_eul_validity(const std::string& eul_type)->bool;

	struct LookAheadParam;
	auto check_look_ahead(vector<double>& axis_begin_pos_vec, BasisFunc& plan, Size num, Size extern_num)->void;
	auto joint_look_ahead_process(BasisFunc& this_plan, LookAheadParam& param, BasisFunc& plan, Size num, Size extern_num, double dt)->int;
	auto carte_look_ahead_process(BasisFunc& this_plan, vector<double>& axis_pos_vec, LookAheadParam& param, BasisFunc& plan, Size num, Size extern_num, double dt)->int;

	class BasisFunc :public core::Object
	{
	public:
		enum Option : std::uint64_t
		{
			NOT_PRINT_CMD_INFO = 0x01ULL << 0,
			NOT_PRINT_EXECUTE_COUNT = 0x01ULL << 1,
			NOT_LOG_CMD_INFO = 0x01ULL << 2,

			NOT_RUN_EXECUTE_FUNCTION = 0x01ULL << 3,
			NOT_RUN_COLLECT_FUNCTION = 0x01ULL << 4,

			WAIT_FOR_EXECUTION = 0x01ULL << 5,
			WAIT_IF_CMD_POOL_IS_FULL = 0x01ULL << 6,
			WAIT_FOR_COLLECTION = 0x01ULL << 7,
		};
		enum MotorOption : std::uint64_t
		{
			USE_TARGET_POS = 0x01ULL << 16,
			USE_TARGET_VEL = 0x01ULL << 17,
			USE_TARGET_TOQ = 0x01ULL << 18,
			USE_OFFSET_VEL = 0x01ULL << 19,
			USE_OFFSET_TOQ = 0x01ULL << 20,

			NOT_CHECK_MODE = 0x01ULL << 22,
			NOT_CHECK_ENABLE = 0x01ULL << 23,
			NOT_CHECK_POS_MIN = 0x01ULL << 24,
			NOT_CHECK_POS_MAX = 0x01ULL << 25,
			NOT_CHECK_POS_CONTINUOUS = 0x01ULL << 26,
			NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER = 0x01ULL << 28,
			NOT_CHECK_POS_FOLLOWING_ERROR = 0x01ULL << 30,

			NOT_CHECK_VEL_MIN = 0x01ULL << 31,
			NOT_CHECK_VEL_MAX = 0x01ULL << 32,
			NOT_CHECK_VEL_CONTINUOUS = 0x01ULL << 33,
			NOT_CHECK_COLLISION = 0x01ULL << 34,
			NOT_CHECK_VEL_FOLLOWING_ERROR = 0x01ULL << 35,

			CHECK_NONE = NOT_CHECK_MODE | NOT_CHECK_ENABLE | NOT_CHECK_POS_MIN | NOT_CHECK_POS_MAX | NOT_CHECK_POS_CONTINUOUS | NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER | NOT_CHECK_POS_FOLLOWING_ERROR |
			NOT_CHECK_VEL_MIN | NOT_CHECK_VEL_MAX | NOT_CHECK_VEL_CONTINUOUS | NOT_CHECK_VEL_FOLLOWING_ERROR,
		};
		enum RetStatus : std::int32_t
		{
			EXECUTED = 1,
			SUCCESS = 0,
			PARSE_EXCEPTION = -1,
			PREPARE_EXCEPTION = -2,
			SERVER_IN_ERROR = -10,
			SERVER_NOT_STARTED = -11,
			COMMAND_POOL_IS_FULL = -12,
			PREPARE_CANCELLED = -40,
			EXECUTE_CANCELLED = -41,
			//以上这些错误状态的错误码需参考errorinfoPool的规定//
			SLAVE_AT_INIT = -101,
			SLAVE_AT_SAFEOP = -102,
			SLAVE_AT_PREOP = -103,
			SLAVE_AT_OP = -104,
			
			SERVO_IN_ERROR = -500,
			MOTION_NOT_ENABLED = -501,
			MOTION_POS_BEYOND_MIN = -502,
			MOTION_POS_BEYOND_MAX = -503,
			MOTION_POS_NOT_CONTINUOUS = -504,
			MOTION_POS_NOT_CONTINUOUS_SECOND_ORDER = -505,
			MOTION_POS_FOLLOWING_ERROR = -506,
			MOTION_VEL_BEYOND_MIN = -507,
			MOTION_VEL_BEYOND_MAX = -508,
			MOTION_VEL_NOT_CONTINUOUS = -509,
			MOTION_VEL_FOLLOWING_ERROR = -510,
			MOTION_INVALID_MODE = -511,
			MOTION_IN_INTRNG = -512,
			MOTION_IN_COLLISION = -513,
			////***新定义的错误码，-1000~-100000之间，需加入map中
			PLAN_OVER_TIME = -1001,
			FORWARD_KINEMATIC_POSITION_FAILED = -1002,
			INVERSE_KINEMATIC_POSITION_FAILED = -1003,
			WRIST_SINGULARITY = -1004,
			SHOULDER_SINGULARITY = -1005,
			ELBOW_SINGULARITY = -1006,
			SINGULARITY = -1007,
			THREE_POINTS_COLLINEAR = -1008,
			TWO_SMALL_LINE = -1009,

			NO_MOVEJ_PLANNER = -1100,
			NO_MOVEL_PLANNER = -1101,
			NO_MOVEC_PLANNER = -1102,
			NO_MOVES_PLANNER = -1103,
			NO_MOVELL_PLANNER = -1104,
			NO_SERVOJ_PLANNER = -1104,

			PROGRAM_EXCEPTION = -2000,
		};
		
		// state machine//
		enum MotionState : std::uint64_t
		{
			OPERATION = 0x01ULL << 0,
			STOPPING = 0x01ULL << 1,
			ERRORSTOP = 0x01ULL << 2,
			EMERGENCYSTOP = 0x01ULL << 3,
			CLEARFUNCPOOL = 0x01ULL << 4,
		};

		// state machine//
		enum ControlMode : std::uint8_t
		{
			POSITION = 0,
			TECH = 1,
			FORCE_TORQUE = 2,
		};


		virtual void prepareNrt(BasisFunc&, int index = 0) {}
		virtual int executeRT(BasisFunc&, int index = 0) { return 0; }
		virtual int executeNRT(BasisFunc&, int index = 0) { return 1; }

		virtual int executeRTMoveLL(BasisFunc&, int index = 0, double* moveLL_pq_set = nullptr, Size moveLL_num = 0, bool moveLL_dec = false, bool first_moveLL = false) { return 0; }
		virtual void collectNrt(BasisFunc&, int index = 0) {}

		auto cmdParams()->const std::map<std::string_view, std::string_view>&;
		std::int64_t& count();
		std::int64_t& nrtCount();
		auto cmdName()->std::string_view;
		auto cmdId()->std::int64_t;
		auto cmdSubId()->Size;
		auto beginGlobalCount()->std::int64_t;
		auto master()->codeit::controller::Master*;
		auto option()->std::uint64_t&;
		auto lastModelDof()->Size&;
		auto lastExternDof()->Size&;
		auto cmdString()->std::string_view;
		auto lout()->core::MsgStream&;
		auto mout()->core::MsgStream&;
		auto command()->core::Command&;
		auto command()const->const core::Command& { return const_cast<std::decay_t<decltype(*this)>*>(this)->command(); }
		auto controlSystem()->codeit::system::ControlSystem*;
		auto model()->codeit::model::Model*;
		auto modelPool()->core::ObjectPool<codeit::model::Model>*;
		auto controller()->codeit::controller::Controller*;
		auto ecMaster()->codeit::controller::EthercatMaster*;
		auto ecController()->codeit::controller::EthercatController*;
		auto ioModelPool()->core::ObjectPool<codeit::model::IOModel>*;

		auto errorPool()->core::ObjectPool<codeit::system::ErrorInfo>*;
		auto sharedPtrForThis()->std::shared_ptr<BasisFunc>;
		auto motorOptions()->std::vector<std::uint64_t>&;
		auto rtStastic()->codeit::controller::Master::RtStasticsData&;
		auto nrtStastic()->codeit::controller::NrtMaster::NrtStasticsData&;

		auto doubleParam(std::string_view param_name)->double;
		auto floatParam(std::string_view param_name)->float;
		auto int32Param(std::string_view param_name)->std::int32_t;
		auto int64Param(std::string_view param_name)->std::int64_t;
		auto uint32Param(std::string_view param_name)->std::uint32_t;
		auto uint64Param(std::string_view param_name)->std::uint64_t;
		auto matrixParam(std::string_view param_name)->core::Matrix;
		auto matrixParam(std::string_view param_name, int m, int n)->core::Matrix;

		auto param()->std::any&;
		auto ret()->std::any&;
		auto setErrMsgRT(const char* msg)->void;
		auto retCode()->std::int32_t&;
		auto retMsg()->const char*;
		auto isNeedRT()->bool;
		auto setIsNeedRT(bool)->void;
		auto nrtControllerPool()->core::ObjectPool<codeit::controller::NrtController>*;
		
		virtual ~BasisFunc();
		explicit BasisFunc(const std::string& name = "BasisFunc");
		CODEIT_REGISTER_TYPE(BasisFunc);
		CODEIT_DECLARE_BIG_FOUR(BasisFunc);
		//


		struct Imp
		{
			std::int64_t count_;
			std::int64_t nrt_count_;
			codeit::model::Model* model_;
			codeit::controller::Master* master_;
			codeit::controller::Controller* controller_;
			codeit::controller::EthercatMaster* ec_master_;
			codeit::controller::EthercatController* ec_controller_;
			codeit::system::ControlSystem* cs_;


			std::weak_ptr<BasisFunc> shared_for_this_;

			std::uint64_t option_{ 0 };
			Size lastModelDof_{ 0 };
			Size lastExternDof_{ 0 };
			std::vector<std::uint64_t> mot_options_;

			std::vector<char> cmd_str_;
			std::string_view cmd_name_;
			std::map<std::string_view, std::string_view> cmd_params_;

			std::int64_t begin_global_count_{ 0 };
			std::uint64_t command_id_{ 0 };
			codeit::controller::Master::RtStasticsData rt_stastic_;
			codeit::controller::NrtMaster::NrtStasticsData nrt_stastic_;


			std::any param;
			std::any ret;
			std::int32_t ret_code;
			char ret_msg[1024]{};
			///////////*************新增变量************////
			double cmdRate;
			Size cmdSubId;
			codeit::controller::VrepMaster* vr_master_;
			codeit::controller::VrepController* vr_controller_;
			core::ObjectPool<codeit::model::IOModel>* iomodel_pool_;
			core::ObjectPool<codeit::model::Model>* model_pool_;
			core::ObjectPool<codeit::controller::NrtController>* nrt_controller_pool_;
			core::ObjectPool<codeit::controller::NrtMaster>* nrt_master_pool_;
			core::ObjectPool<codeit::system::ErrorInfo>* errorinfo_pool_;
			bool is_need_RT_{ true };
			

			template<typename Type>
			auto inline getType(std::string_view param)->Type
			{
				Type ret;
				auto value = cmd_params_.at(param);
				auto result = std::from_chars(value.data(), value.data() + value.size(), ret);
				if (result.ec == std::errc::invalid_argument) { THROW_FILE_LINE("invalid argument for param:" + std::string(param)); }
				return ret;
			}
		};
		core::ImpPtr<Imp> imp_;

		friend class system::ControlSystem;
	};

	class FuncRoot :public core::Object
	{
	public:
		auto funcPool()->core::ObjectPool<BasisFunc>&;
		auto funcPool()const->const core::ObjectPool<BasisFunc>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->funcPool(); }
		auto funcParser()->core::CommandParser&;
		auto init()->void;

		virtual ~FuncRoot();
		explicit FuncRoot(const std::string& name = "func_root");
		CODEIT_REGISTER_TYPE(FuncRoot);
		CODEIT_DECLARE_BIG_FOUR(FuncRoot);

	private:
		struct Imp { core::CommandParser parser_; };
		core::ImpPtr<Imp> imp_;
	};

#define SELECT_MOTOR_STRING \
		"		<UniqueParam default=\"all\">"\
		"			<Param name=\"all\" abbreviation=\"a\"/>"\
		"			<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"\
		"		</UniqueParam>"

#define SET_INPUT_MOVEMENT_STRING \
		"		<Param name=\"pos\" default=\"0.5\"/>"\
		"		<Param name=\"acc\" default=\"0.1\"/>"\
		"		<Param name=\"vel\" default=\"0.1\"/>"\
		"		<Param name=\"jerk\" default=\"10\"/>"\
		"		<Param name=\"dec\" default=\"0.1\"/>"

#define SET_TOOL_STRING \
		"		<UniqueParam default=\"tool\">"\
		"			<Param name=\"tool\" default=\"tool0\"/>"\
		"			<Param name=\"tool_var\" default=\"tool0\"/>"\
		"		</UniqueParam>"

#define SET_WOBJ_STRING \
		"		<UniqueParam default=\"wobj\">"\
		"			<Param name=\"wobj\" default=\"wobj0\"/>"\
		"			<Param name=\"wobj_var\" default=\"wobj0\"/>"\
		"		</UniqueParam>"

#define SET_ROBTARGET_STRING \
		"		<UniqueParam default=\"robottarget\">"\
		"			<Param name=\"robottarget\" default=\"P0\"/>"\
		"			<Param name=\"robottarget_var\" default=\"P0\"/>"\
		"		</UniqueParam>"

#define SET_ROBTARGET_SET_STRING \
		"		<UniqueParam default=\"robottarget_set\">"\
		"			<Param name=\"robottarget_set\" default=\"{P0}\"/>"\
		"			<Param name=\"robottarget_var_set\" default=\"{P0}\"/>"\
		"		</UniqueParam>"


#define SET_MID_ROBTARGET_STRING \
		"		<UniqueParam default=\"mid_robottarget\">"\
		"			<Param name=\"mid_robottarget\" default=\"P0\"/>"\
		"			<Param name=\"mid_robottarget_var\" default=\"P0\"/>"\
		"		</UniqueParam>"

#define SET_JOINTTARGET_STRING \
		"		<UniqueParam default=\"jointtarget\">"\
		"			<Param name=\"jointtarget\" default=\"P1\"/>"\
		"			<Param name=\"jointtarget_var\" default=\"P0\"/>"\
		"		</UniqueParam>"

#define SET_ZEROCOMP_STRING \
		"		<UniqueParam default=\"zerocomp\">"\
		"			<Param name=\"zerocomp\" default=\"zero_comp0\"/>"\
		"			<Param name=\"zerocomp_var\" default=\"zero_comp0\"/>"\
		"		</UniqueParam>"


#define SET_SPEED_STRING \
		"		<UniqueParam default=\"speed\">"\
		"			<Param name=\"speed\" default=\"v100\"/>"\
		"			<Param name=\"speed_var\" default=\"v100\"/>"\
		"		</UniqueParam>"

#define SET_ZONE_STRING \
		"		<UniqueParam default=\"zone\">"\
		"			<Param name=\"zone\" default=\"fine\"/>"\
		"			<Param name=\"zone_var\" default=\"fine\"/>"\
		"		</UniqueParam>"

#define SET_LOAD_STRING \
		"		<UniqueParam default=\"load\">"\
		"			<Param name=\"load\" default=\"load0\"/>"\
		"			<Param name=\"load_var\" default=\"load0\"/>"\
		"		</UniqueParam>"


#define POSITIVE_CHECK(p)\
if(p<=1.0e-10)  THROW_FILE_LINE("") 

#define NEGATIVE_CHECK(p)\
if(p>=-1.0e-10)  THROW_FILE_LINE("") 


}

#endif
