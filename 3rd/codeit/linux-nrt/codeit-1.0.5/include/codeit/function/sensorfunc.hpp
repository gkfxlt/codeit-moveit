#ifndef CODEIT_SENSOR_FUNCTION_H_
#define CODEIT_SENSOR_FUNCTION_H_
#include<codeit/function/basefunc.hpp>
namespace codeit::function {
	class VisualServo : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int index = 0)->int { return 0; }
		auto virtual collectNrt(BasisFunc&, int)->void; 
		virtual ~VisualServo();
		explicit VisualServo(const std::string& name = "VisualServo");
		CODEIT_REGISTER_TYPE(VisualServo);
		CODEIT_DECLARE_BIG_FOUR(VisualServo);
	};

	
}



#endif

