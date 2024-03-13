#ifndef CODEIT_TRIGG_FUNCTION_H_
#define CODEIT_TRIGG_FUNCTION_H_
#include<codeit/function/basefunc.hpp>
namespace codeit::function {
	
#define SET_BREAK_STRING \
		"		<UniqueParam default=\"until_nothing\">"\
		"			<Param name=\"until_time\" default=\"1000\"/>"\
		"			<Param name=\"until_nothing\" />"\
		"			<GroupParam>"\
		"				<Param name=\"break_master\" default=\"com\"/>"\
		"				<Param name=\"di_name\" default=\"com_di0\"/>"\
		"				<Param name=\"di_value\" default=\"0\"/>"\
		"			</GroupParam>"\
		"		</UniqueParam>"

#define SET_TRIGGLE_STRING \
		"		<UniqueParam default=\"triggle_nothing\">"\
		"			<Param name=\"triggle_nothing\" />"\
		"			<GroupParam>"\
		"				<Param name=\"triggle_time\" default=\"1000\"/>"\
		"				<Param name=\"triggle_master\" default=\"com\"/>"\
		"				<Param name=\"do_name\" default=\"com_do0\"/>"\
		"				<Param name=\"do_value\" default=\"0\"/>"\
		"			</GroupParam>"\
		"		</UniqueParam>"


#define BREAK_INIT\
	if (cmd_param.first == "until_time")\
	{\
		param.until_time = doubleParam(cmd_param.first);\
		param.last_count = int64Param("last_count");\
		param.break_flag = true;\
	}\
	else if (cmd_param.first == "break_master")\
	{\
			param.break_master = std::string(cmdParams().at(cmd_param.first));\
			param.last_count = int64Param("last_count");\
			param.until_time = 1.0e15;\
			param.break_flag = true;\
	}\
	else if (cmd_param.first == "di_name")\
	{\
		param.di_name = std::string(cmdParams().at(cmd_param.first));\
	}\
	else if (cmd_param.first == "di_value")\
	{\
		param.di_value = int32Param(cmd_param.first);\
	}\
	else if (cmd_param.first == "until_nothing")\
	{\
		param.break_flag = false;\
	}

	struct SetBreakParam
	{
		////Motion Break////
		double until_time{ 0 };
		std::string break_master, di_name;
		bool di_value;
		bool break_flag{ true };
		std::int64_t last_count;
	};

#define TRIGGLE_INIT\
	if (cmd_param.first == "triggle_time")\
	{\
		param.triggle_time = doubleParam(cmd_param.first);\
		param.triggle_flag = true;\
	}\
	else if (cmd_param.first == "triggle_master")\
	{\
			param.triggle_master = std::string(cmdParams().at(cmd_param.first));\
	}\
	else if (cmd_param.first == "do_name")\
	{\
		param.do_name = std::string(cmdParams().at(cmd_param.first));\
	}\
	else if (cmd_param.first == "do_value")\
	{\
		param.do_value = int32Param(cmd_param.first);\
	}\
	else if (cmd_param.first == "triggle_nothing")\
	{\
		param.triggle_flag = false;\
	}
	struct SetTriggleParam
	{
		////Motion Triggle////
		double triggle_time{ 0 };
		std::string triggle_master, do_name;
		bool do_value;
		bool triggle_flag{ true };
	};


	class TriggAbsJ :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);
		virtual int executeRT(BasisFunc&, int);
		virtual void collectNrt(BasisFunc&, int);
		virtual ~TriggAbsJ();
		explicit TriggAbsJ(const std::string& name = "TriggAbsJ");
		CODEIT_REGISTER_TYPE(TriggAbsJ);
		CODEIT_DECLARE_BIG_FOUR(TriggAbsJ);
	};

	class TriggJ :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);
		virtual int executeRT(BasisFunc&, int);
		virtual void collectNrt(BasisFunc&, int);
		virtual ~TriggJ();
		explicit TriggJ(const std::string& name = "TriggJ");
		CODEIT_REGISTER_TYPE(TriggJ);
		CODEIT_DECLARE_BIG_FOUR(TriggJ);
	};

	class TriggL :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);
		virtual int executeRT(BasisFunc&, int);
		virtual void collectNrt(BasisFunc&, int);
		virtual ~TriggL();
		explicit TriggL(const std::string& name = "TriggL");
		CODEIT_REGISTER_TYPE(TriggL);
		CODEIT_DECLARE_BIG_FOUR(TriggL);
	};

	class TriggC :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);
		virtual int executeRT(BasisFunc&, int);
		virtual void collectNrt(BasisFunc&, int);
		virtual ~TriggC();
		explicit TriggC(const std::string& name = "TriggC");
		CODEIT_REGISTER_TYPE(TriggC);
		CODEIT_DECLARE_BIG_FOUR(TriggC);
	};

	class TriggS :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);
		virtual int executeRT(BasisFunc&, int);
		virtual void collectNrt(BasisFunc&, int);
		virtual ~TriggS();
		explicit TriggS(const std::string& name = "TriggS");
		CODEIT_REGISTER_TYPE(TriggS);
		CODEIT_DECLARE_BIG_FOUR(TriggS);
	};
}
#endif
