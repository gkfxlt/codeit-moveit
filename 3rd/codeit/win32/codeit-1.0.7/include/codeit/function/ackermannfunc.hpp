#ifndef CODEIT_FUNC_NRTACKERMANN_H_
#define CODEIT_FUNC_NRTACKERMANN_H_

#include <codeit/function/basefunc.hpp>
namespace codeit::function {
	class NrtAKMMove :public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int { return 1; };
		auto virtual executeNRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		double lastTargetData[2] = { 0.0 };
		double refrenceData[2] = { 0.0 };
		double k_linerVel = 0.0;
		double k_angle = 0.0;
		double targetData[2] = { 0.0, 0.0 };
		int times = 10;
		int counter = 0;
		double init_pos;
		double last_time = 0;

		virtual ~NrtAKMMove();
		explicit NrtAKMMove(const std::string& name = "NrtAKMMove");
		CODEIT_REGISTER_TYPE(NrtAKMMove);
		CODEIT_DECLARE_BIG_FOUR(NrtAKMMove);
	};

}


#endif
