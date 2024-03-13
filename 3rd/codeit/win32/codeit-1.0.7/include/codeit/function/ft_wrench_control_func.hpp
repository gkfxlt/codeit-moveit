#ifndef CODEIT_FT_WRENCH_CONTROL_FUNC_H_
#define CODEIT_FT_WRENCH_CONTROL_FUNC_H_

#include <codeit/function/basefunc.hpp>

namespace codeit::system {
class ControlSystem;
} // namespace codeit::system

namespace codeit::function {

class ImpedanceMoveL : public BasisFunc {
public:
	auto virtual prepareNrt(BasisFunc&, int) -> void;
	auto virtual executeRT(BasisFunc&, int) -> int;
	auto virtual collectNrt(BasisFunc&, int) -> void;

	virtual ~ImpedanceMoveL();
	explicit ImpedanceMoveL(const std::string& name = "ImpedanceMoveL");
	CODEIT_REGISTER_TYPE(ImpedanceMoveL);
	CODEIT_DECLARE_BIG_FOUR(ImpedanceMoveL);
};

class WrenchPoseMoveL : public BasisFunc {
public:
	auto virtual prepareNrt(BasisFunc&, int) -> void;
	auto virtual executeRT(BasisFunc&, int) -> int;
	auto virtual collectNrt(BasisFunc&, int) -> void;

	virtual ~WrenchPoseMoveL();
	explicit WrenchPoseMoveL(const std::string& name = "WrenchPoseMoveL");
	CODEIT_REGISTER_TYPE(WrenchPoseMoveL);
	CODEIT_DECLARE_BIG_FOUR(WrenchPoseMoveL);
};

class WrenchPose : public BasisFunc {
public:
	auto virtual prepareNrt(BasisFunc&, int) -> void;
	auto virtual executeRT(BasisFunc&, int) -> int;
	auto virtual collectNrt(BasisFunc&, int) -> void;

	virtual ~WrenchPose();
	explicit WrenchPose(const std::string& name = "WrenchPose");
	CODEIT_REGISTER_TYPE(WrenchPose);
	CODEIT_DECLARE_BIG_FOUR(WrenchPose);
};

class WrenchPoseV2 : public BasisFunc {
public:
	auto virtual prepareNrt(BasisFunc&, int) -> void;
	auto virtual executeRT(BasisFunc&, int) -> int;
	auto virtual collectNrt(BasisFunc&, int) -> void;

	virtual ~WrenchPoseV2();
	explicit WrenchPoseV2(const std::string& name = "WrenchPoseV2");
	CODEIT_REGISTER_TYPE(WrenchPoseV2);
	CODEIT_DECLARE_BIG_FOUR(WrenchPoseV2);
};

};

#endif // !FT_WRENCH_CONTROL_FUNC_H_
