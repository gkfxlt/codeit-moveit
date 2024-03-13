#ifndef CODEIT_MODEL_DVRK_SLAVE_H_
#define CODEIT_MODEL_DVRK_SLAVE_H_

#include <array>
#include <codeit/model/model_solver.hpp>

namespace codeit::model
{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	struct DvrkSlaveParam
	{
		// DH PARAM //
		double beta0{ 0.0 };
		double beta1{ 0.0 };
		double beta2{ 0.0 };
		double Lr2rcm{ 0.0 };
		double Lrcm1{ 0.0 };
		double Lrcm2{ 0.0 };
		double Lrcm3{ 0.0 };
		double Lrcm4{ 0.0 };
		double Lrcm5{ 0.0 };
		double Lend2rcm{ 0.0 };
		double Lwrist{ 0.0 };
		// TOOL 0, by default is 321 type
		double tool0_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::string tool0_pe_type;

		// BASE wrt REF, by default is 321 type 
		double base2ref_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::string base2ref_pe_type = "321";

		// inertia vector, size must be 7
		std::vector<std::array<double, 10> > iv_vec;

		// mot friction vector, size must be 7
		std::vector<std::array<double, 3> > mot_frc_vec;

		// 驱动在零位处的偏移，以及系数
		double mp_offset[6];// mp_real = (mp_theoretical - mp_offset) * mp_factor
		double mp_factor[6];
	};
	auto createDvrkSlaveModel(const DvrkSlaveParam& param, std::string name)->std::unique_ptr<codeit::model::Model>;

	class DvrkSlaveInverseKinematicSolver :public codeit::model::InverseKinematicSolver
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;
		auto setWhichRoot(int root_of_0_to_7)->void;
		auto whichRoot()->int;
		auto setAxisAngle(double axis_angle)->void;

		virtual ~DvrkSlaveInverseKinematicSolver() = default;
		explicit DvrkSlaveInverseKinematicSolver(const std::string& name = "dvrk_slave_inverse_solver");
		CODEIT_REGISTER_TYPE(DvrkSlaveInverseKinematicSolver);
		CODEIT_DECLARE_BIG_FOUR(DvrkSlaveInverseKinematicSolver);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};

	class DvrkSlaveInverseDynamicSolver :public codeit::model::InverseDynamicSolver
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto virtual allocateMemory()->void override;
		auto virtual dynAccAndFce()->void override;
		auto dynSubRegressorMatrix(const double* pos)->void;
	
		virtual ~DvrkSlaveInverseDynamicSolver() = default;
		explicit DvrkSlaveInverseDynamicSolver(const std::string& name = "dvrk_slave_dynamic_solver");
		CODEIT_REGISTER_TYPE(DvrkSlaveInverseDynamicSolver);
		CODEIT_DECLARE_BIG_FOUR(DvrkSlaveInverseDynamicSolver);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};

	///
	/// @}
}

#endif
