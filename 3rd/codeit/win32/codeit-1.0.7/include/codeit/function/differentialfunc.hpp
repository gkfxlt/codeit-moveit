#ifndef CODEIT_FUNC_NRTDIFFERENTIAL_H_
#define CODEIT_FUNC_NRTDIFFERENTIAL_H_

#include <codeit/function/basefunc.hpp>

namespace codeit::function {
	class NrtDiffMove :public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeNRT(BasisFunc&, int)->int;
		auto virtual executeRT(BasisFunc&, int)->int { return 1; };
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~NrtDiffMove();
		explicit NrtDiffMove(const std::string& name = "NrtDiffMove");
		CODEIT_REGISTER_TYPE(NrtDiffMove);
		CODEIT_DECLARE_BIG_FOUR(NrtDiffMove);

	private:
		double lastTargetSpeed[6] = { 0.0 };
		double refrenceSpeed[6] = { 0.0 };
		double k_linerVelX = 0.0;
		double k_angleVel = 0.0;
		double targetspeed[6] = { 0.0 };
		int times = 10;
		int counter = 0;

		clock_t last_time = 0;
	};

}

#endif