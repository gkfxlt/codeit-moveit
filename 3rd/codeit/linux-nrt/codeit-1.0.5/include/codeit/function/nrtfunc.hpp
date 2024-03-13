#ifndef CODEIT_NRT_FUNCTION_H_
#define CODEIT_NRT_FUNCTION_H_
#include<codeit/function/basefunc.hpp>

namespace codeit::system { class ControlSystem; }
namespace codeit::function {


	class NrtEnable :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);
		auto virtual executeRT(BasisFunc&, int)->int { return 1; };
		auto virtual executeNRT(BasisFunc&, int)->int;
		virtual void collectNrt(BasisFunc&, int);
		virtual ~NrtEnable();
		explicit NrtEnable(const std::string& name = "NrtEnable");
		CODEIT_REGISTER_TYPE(NrtEnable);
		CODEIT_DECLARE_BIG_FOUR(NrtEnable);
	};

	class NrtDisable : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int { return 1; };
		auto virtual executeNRT(BasisFunc&, int)->int;
		virtual ~NrtDisable();
		explicit NrtDisable(const std::string& name = "NrtDisable");
		CODEIT_REGISTER_TYPE(NrtDisable);
		CODEIT_DECLARE_BIG_FOUR(NrtDisable);
	};

	class NrtMode : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int { return 1; };
		auto virtual executeNRT(BasisFunc&, int)->int;
		virtual ~NrtMode();
		explicit NrtMode(const std::string& name = "NrtMode");
		CODEIT_REGISTER_TYPE(NrtMode);
		CODEIT_DECLARE_BIG_FOUR(NrtMode);
	};

	class NrtRecover : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int { return 1; };
		auto virtual executeNRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~NrtRecover();
		explicit NrtRecover(const std::string& name = "NrtRecover");
		CODEIT_REGISTER_TYPE(NrtRecover);
		CODEIT_DECLARE_BIG_FOUR(NrtRecover);
	};

	class NrtServoV : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int { return 1; };
		auto virtual executeNRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~NrtServoV();
		explicit NrtServoV(const std::string& name = "NrtServoV");
		CODEIT_REGISTER_TYPE(NrtServoV);
		CODEIT_DECLARE_BIG_FOUR(NrtServoV);

	};

	class NrtMovePP : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int { return 1; };
		auto virtual executeNRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~NrtMovePP();
		explicit NrtMovePP(const std::string& name = "NrtMovePP");
		CODEIT_REGISTER_TYPE(NrtMovePP);
		CODEIT_DECLARE_BIG_FOUR(NrtMovePP);

	};

	class NrtStopV : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int { return 1; };
		auto virtual executeNRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~NrtStopV();
		explicit NrtStopV(const std::string& name = "NrtStopV");
		CODEIT_REGISTER_TYPE(NrtStopV);
		CODEIT_DECLARE_BIG_FOUR(NrtStopV);

	};

	class NrtStop :public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int { return 0; };
		auto virtual executeNRT(BasisFunc&, int)->int { return 0; }
		virtual void collectNrt(BasisFunc&, int);
		virtual ~NrtStop();
		explicit NrtStop(const std::string& name = "NrtStop");
		CODEIT_REGISTER_TYPE(NrtStop);
	};


	class NrtShow :public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int { return 0; };
		auto virtual executeNRT(BasisFunc&, int)->int { return 0; }
		virtual ~NrtShow();
		explicit NrtShow(const std::string& name = "NrtShow");
		CODEIT_REGISTER_TYPE(NrtShow);
		CODEIT_DECLARE_BIG_FOUR(NrtShow);
	};

	class NrtTry : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int { return 1; };
		auto virtual executeNRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~NrtTry();
		explicit NrtTry(const std::string& name = "NrtTry");
		CODEIT_REGISTER_TYPE(NrtTry);
		CODEIT_DECLARE_BIG_FOUR(NrtTry);

	};
}
//
//
#endif


