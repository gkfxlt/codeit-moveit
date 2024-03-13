#ifndef CODEIT_FTPLAN_FUNCTION_H_
#define CODEIT_FTPLAN_FUNCTION_H_
#include<codeit/function/basefunc.hpp>

namespace codeit::system { class ControlSystem; }
namespace codeit::function {

	double* getInteractionFT(const double* pm_now, const codeit::controller::FTsensor& ft_sensor, const double* load, double* interaction_ft);

	class DragFtInCartesian : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~DragFtInCartesian();
		explicit DragFtInCartesian(const std::string& name = "DragFtInCartesian");
		CODEIT_REGISTER_TYPE(DragFtInCartesian);
		CODEIT_DECLARE_BIG_FOUR(DragFtInCartesian);

	};

	class DragFtInJoint : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~DragFtInJoint();
		explicit DragFtInJoint(const std::string& name = "DragFtInJoint");
		CODEIT_REGISTER_TYPE(DragFtInJoint);
		CODEIT_DECLARE_BIG_FOUR(DragFtInJoint);

	};

	
}
#endif