#ifndef CODEIT_DYNAMIC_SERIAL_3_AXIS_H_
#define CODEIT_DYNAMIC_SERIAL_3_AXIS_H_

#include <array>
#include <codeit/model/model_solver.hpp>

namespace codeit::model
{
	struct Serial3Param
	{
		// DH PARAM //
		double a1{ 0.5 };
		double a2{ 0.5 };
		double a3{ 0.5 };

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
	auto createModelSerial3Axis(const Serial3Param &param)->std::unique_ptr<codeit::model::Model>;

	class Serial3InverseKinematicSolver :public codeit::model::InverseKinematicSolver
	{
	public:
		auto virtual saveXml(core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement &xml_ele)->void override;
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;
		auto setWhichRoot(int root_of_0_to_4)->void;
		auto setPosEE(const double *ee_pos)->void;

		virtual ~Serial3InverseKinematicSolver();
		explicit Serial3InverseKinematicSolver(const Serial3Param &param, const std::string &name = "puma_inverse_solver");
		CODEIT_REGISTER_TYPE(Serial3InverseKinematicSolver);
		CODEIT_DECLARE_BIG_FOUR(Serial3InverseKinematicSolver);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
}

#endif
