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

	class ForceMode :public BasisFunc {
	public:
		virtual void prepareNrt(BasisFunc&, int);
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~ForceMode();
		explicit ForceMode(const std::string& name = "ForceMode");
		CODEIT_REGISTER_TYPE(ForceMode);
		CODEIT_DECLARE_BIG_FOUR(ForceMode);
	};

	class ForceModeInCSP : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~ForceModeInCSP();
		explicit ForceModeInCSP(const std::string& name = "ForceModeInCSP");
		CODEIT_REGISTER_TYPE(ForceModeInCSP);
		CODEIT_DECLARE_BIG_FOUR(ForceModeInCSP);
	};

}




#endif
