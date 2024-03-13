#ifndef CODEIT_DYNAMIC_UR_H_
#define CODEIT_DYNAMIC_UR_H_

#include <codeit/model/model_solver.hpp>

namespace codeit::model
{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	struct UrParam
	{
		// DH PARAM //
		double L1{ 0 };
		double L2{ 0 };
		double W1{ 0 };
		double W2{ 0 };
		double H1{ 0 };
		double H2{ 0 };
		
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
	};
	auto createModelUr(const UrParam& param, std::string name = "model")->std::unique_ptr<codeit::model::Model>;

	class UrInverseKinematicSolver :public codeit::model::InverseKinematicSolver
	{
	public:
		auto virtual saveXml(core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement &xml_ele)->void override;
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;
		auto setWhichRoot(int root_of_0_to_7)->void;

		virtual ~UrInverseKinematicSolver() = default;
		explicit UrInverseKinematicSolver(const std::string& name = "ur_inverse_solver");
		CODEIT_REGISTER_TYPE(UrInverseKinematicSolver);
		CODEIT_DECLARE_BIG_FOUR(UrInverseKinematicSolver);
		
	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	///
	/// @}
}

#endif
