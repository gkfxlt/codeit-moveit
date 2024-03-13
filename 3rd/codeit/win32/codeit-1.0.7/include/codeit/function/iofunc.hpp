#ifndef CODEIT_IO_FUNCTION_H_
#define CODEIT_IO_FUNCTION_H_
#include<codeit/function/basefunc.hpp>

namespace codeit::system { class ControlSystem; }
namespace codeit::function {

	class DOSet : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;
		virtual ~DOSet();
		explicit DOSet(const std::string& name = "DOSet");
		CODEIT_REGISTER_TYPE(DOSet);
		CODEIT_DECLARE_BIG_FOUR(DOSet);
	};

	class DIGet : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;
		
		explicit DIGet(const std::string& name = "DIGet");
		virtual ~DIGet();
		CODEIT_REGISTER_TYPE(DIGet);
		CODEIT_DECLARE_BIG_FOUR(DIGet);
	};

	class DOPulse : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;
		explicit DOPulse(const std::string& name = "DOPulse");
		virtual ~DOPulse();
		CODEIT_REGISTER_TYPE(DOPulse);
		CODEIT_DECLARE_BIG_FOUR(DOPulse);
	};


	class ReplayCmd : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;
		explicit ReplayCmd(const std::string& name = "ReplayCmd");
		virtual ~ReplayCmd();
		CODEIT_REGISTER_TYPE(ReplayCmd);
		CODEIT_DECLARE_BIG_FOUR(ReplayCmd);
	};

}


#endif