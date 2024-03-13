#ifndef CODEIT_QUAD_FUNCTION_H_
#define CODEIT_QUAD_FUNCTION_H_
#include<codeit/function/basefunc.hpp>

namespace codeit::system { class ControlSystem; }
namespace codeit::function {

	class QuadMoveCos : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~QuadMoveCos();
		explicit QuadMoveCos(const std::string& name = "QuadMoveCos");
		CODEIT_REGISTER_TYPE(QuadMoveCos);
		CODEIT_DECLARE_BIG_FOUR(QuadMoveCos);

	};


}
#endif