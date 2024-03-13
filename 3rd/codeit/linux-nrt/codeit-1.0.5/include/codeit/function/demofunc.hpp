#ifndef CODEIT_DEMO_FUNCTION_H_
#define CODEIT_DEMO_FUNCTION_H_
#include<codeit/function/basefunc.hpp>

namespace codeit::system { class ControlSystem; }
namespace codeit::function {
	
	class MoveSine : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~MoveSine();
		explicit MoveSine(const std::string& name = "MoveSine");
		CODEIT_REGISTER_TYPE(MoveSine);
		CODEIT_DECLARE_BIG_FOUR(MoveSine);

	};


	class MoveReverse : public BasisFunc {
	public:
		auto virtual prepareNrt(BasisFunc &, int) -> void;
		auto virtual executeRT(BasisFunc &, int) -> int;
		auto virtual collectNrt(BasisFunc &, int) -> void;

		virtual ~MoveReverse();
		explicit MoveReverse(const std::string &name = "MoveReverse");
		CODEIT_REGISTER_TYPE(MoveReverse);
		CODEIT_DECLARE_BIG_FOUR(MoveReverse);
	};

}
#endif