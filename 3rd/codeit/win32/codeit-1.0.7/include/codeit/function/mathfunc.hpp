#ifndef CODEIT_MATH_FUNCTION_H_
#define CODEIT_MATH_FUNCTION_H_
#include<codeit/function/basefunc.hpp>
namespace codeit::function {
	class Sin : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		virtual ~Sin();
		explicit Sin(const std::string& name = "Sin");
		CODEIT_REGISTER_TYPE(Sin);
		CODEIT_DECLARE_BIG_FOUR(Sin);
	};




}
#endif
