#ifndef CODEIT_CAN_FUNCTION_H_
#define CODEIT_CAN_FUNCTION_H_
#include<codeit/function/basefunc.hpp>

namespace codeit::system { class ControlSystem; }
namespace codeit::function {

	class ServoV : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		//auto virtual executeNRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~ServoV();
		explicit ServoV(const std::string& name = "ServoV");
		CODEIT_REGISTER_TYPE(ServoV);
		CODEIT_DECLARE_BIG_FOUR(ServoV);

	};

	class StopV : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		//auto virtual executeNRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~StopV();
		explicit StopV(const std::string& name = "StopV");
		CODEIT_REGISTER_TYPE(StopV);
		CODEIT_DECLARE_BIG_FOUR(StopV);

	};


}
//
//
#endif

