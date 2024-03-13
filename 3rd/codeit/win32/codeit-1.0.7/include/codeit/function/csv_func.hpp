#ifndef CODEIT_CSV_FUNC__H_
#define CODEIT_CSV_FUNC__H_

#include <codeit/function/basefunc.hpp>

namespace codeit::function {

	class ArmMoveCSV : public BasisFunc {
	public:
		auto virtual prepareNrt(BasisFunc&, int) -> void;
		auto virtual executeRT(BasisFunc&, int) -> int;
		auto virtual collectNrt(BasisFunc&, int) -> void;

		virtual ~ArmMoveCSV();
		explicit ArmMoveCSV(const std::string& name = "ArmMoveCSV");
		CODEIT_REGISTER_TYPE(ArmMoveCSV);
		CODEIT_DECLARE_BIG_FOUR(ArmMoveCSV);
	};

	class JointCSV : public BasisFunc {
	public:
		auto virtual prepareNrt(BasisFunc&, int) -> void;
		auto virtual executeRT(BasisFunc&, int) -> int;
		auto virtual collectNrt(BasisFunc&, int) -> void;

		virtual ~JointCSV();
		explicit JointCSV(const std::string& name = "JointCSV");
		CODEIT_REGISTER_TYPE(JointCSV);
		CODEIT_DECLARE_BIG_FOUR(JointCSV);
	};

} //namespace codeit::function
#endif