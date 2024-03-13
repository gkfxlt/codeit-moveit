#pragma once

#include <codeit/model/model_solver.hpp>

namespace codeit::model {

	struct MobileRobotBetaParam {
		double r;//wheel radius
		double l;//distance between front and rear axis;
	};

	auto createModelMobileRobotBeta(const MobileRobotBetaParam &param)->std::unique_ptr<codeit::model::Model>;

	class MobileRobotBetaInverseKinematicSolver :public InverseKinematicSolver {
		public:
			virtual ~MobileRobotBetaInverseKinematicSolver() = default;
			explicit MobileRobotBetaInverseKinematicSolver(const std::string &name = "mobile_robot_beta_inverse_solver");
			auto virtual allocateMemory()->void override;
			auto virtual kinPos()->int override;

			auto virtual saveXml(core::XmlElement &xml_ele) const->void override;
			auto virtual loadXml(const core::XmlElement &xml_ele)->void override;

			CODEIT_REGISTER_TYPE(MobileRobotBetaInverseKinematicSolver);
			CODEIT_DECLARE_BIG_FOUR(MobileRobotBetaInverseKinematicSolver);
		private:
			struct Imp;
			core::ImpPtr<Imp> imp_;
	};

}