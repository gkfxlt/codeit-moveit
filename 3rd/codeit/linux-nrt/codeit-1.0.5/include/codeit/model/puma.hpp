#ifndef CODEIT_DYNAMIC_PUMA_H_
#define CODEIT_DYNAMIC_PUMA_H_

#include <array>
#include <codeit/model/model_solver.hpp>

namespace codeit::model
{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	struct PumaParam
	{
		// DH PARAM //
		double d1{ 0.0 };
		double a1{ 0.0 };
		double a2{ 0.0 };
		double d3{ 0.0 };
		double a3{ 0.0 };
		double d4{ 0.0 };

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
	auto createModelPuma(const PumaParam &param,std::string name="model")->std::unique_ptr<codeit::model::Model>;

	class PumaInverseKinematicSolver :public codeit::model::InverseKinematicSolver
	{
	public:
		auto virtual saveXml(core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement &xml_ele)->void override;		
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;
		auto setWhichRoot(int root_of_0_to_7)->void;

		virtual ~PumaInverseKinematicSolver() = default;
		explicit PumaInverseKinematicSolver(const std::string &name = "puma_inverse_solver");
		CODEIT_REGISTER_TYPE(PumaInverseKinematicSolver);
		CODEIT_DECLARE_BIG_FOUR(PumaInverseKinematicSolver);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	///
	/// @}
}

#endif
