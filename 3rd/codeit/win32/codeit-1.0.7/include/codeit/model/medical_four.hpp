#ifndef CODEIT_MODEL_MEDICAL_FOUR_H_
#define CODEIT_MODEL_MEDICAL_FOUR_H_

#include <codeit/model/model_solver.hpp>

namespace codeit::model {
	/// @defgroup dynamic_model_group
	/// @{
	///
	struct MedicalFourParam {
		// TOOL 0, by default is 321 type
		double tool0_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::string tool0_pe_type;

		// BASE wrt REF, by default is 321 type
		double base2ref_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::string base2ref_pe_type;

		// inertia vector, size must be 6
		std::vector<std::array<double, 10> > iv_vec;

		// mot friction vector, size must be 6
		std::vector<std::array<double, 3> > mot_frc_vec;

		// zero offset of the driver
		double mp_offset[3]; 
		double mp_factor[3];
	};

	auto createModelMedicalFour(const MedicalFourParam &param, std::string name = "model")->std::unique_ptr<codeit::model::Model>;
	auto medicalFourInverse(MedicalFourParam &param, const double *ee_pm, double axis_angle, int which_root, double *input)->bool;

	class MedicalFourInverseKinematicSolver : public codeit::model::InverseKinematicSolver {
	public:
		auto virtual saveXml(core::XmlElement &xml_ele) const -> void override;
		auto virtual loadXml(const core::XmlElement &xml_ele) -> void override;
		auto virtual allocateMemory() -> void override;
		auto virtual kinPos() -> int override;
		auto setWhichRoot(int root_of_0_to_7)->void;

		virtual ~MedicalFourInverseKinematicSolver() = default;
		explicit MedicalFourInverseKinematicSolver(const std::string &name = "ur_three_inverse_solver");
		CODEIT_REGISTER_TYPE(MedicalFourInverseKinematicSolver);
		CODEIT_DECLARE_BIG_FOUR(MedicalFourInverseKinematicSolver);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};

	///
	/// @}
}

#endif