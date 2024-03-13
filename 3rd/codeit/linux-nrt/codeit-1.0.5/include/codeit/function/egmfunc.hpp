#ifndef CODEIT_EGM_FUNCTION_H_
#define CODEIT_EGM_FUNCTION_H_
#include<codeit/function/basefunc.hpp>

namespace codeit::system { class ControlSystem; }
namespace codeit::function {

	class FCPressL : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int index)->void;
		explicit FCPressL(const std::string& name = "FCPressL");
		CODEIT_REGISTER_TYPE(FCPressL);
		CODEIT_DECLARE_BIG_FOUR(FCPressL);
	};








}


#endif