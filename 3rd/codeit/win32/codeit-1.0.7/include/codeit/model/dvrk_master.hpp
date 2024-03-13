#ifndef CODEIT_MODEL_DVRK_MASTER_H_
#define CODEIT_MODEL_DVRK_MASTER_H_

#include <array>
#include <codeit/model/model_solver.hpp>

namespace codeit::model
{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	struct DvrkMasterParam
	{
		std::string arm_type = "left";
		// DH PARAM //
		double Larm1{ 0.0 };
		double Larm2{ 0.0 };
		double Larm3{ 0.0 };
		double Lh{ 0.0 };
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
		double mp_offset[7];// mp_real = (mp_theoretical - mp_offset) * mp_factor
		double mp_factor[7];
	};
	auto createDvrkMasterModel(const DvrkMasterParam& param, std::string name)->std::unique_ptr<codeit::model::Model>;

	class DvrkMasterInverseKinematicSolver :public codeit::model::InverseKinematicSolver
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;
		auto setWhichRoot(int root_of_0_to_7)->void;
		auto whichRoot()->int;
		auto setAxisAngle(double axis_angle)->void;

		virtual ~DvrkMasterInverseKinematicSolver() = default;
		explicit DvrkMasterInverseKinematicSolver(const std::string& name = "dvrk_master_inverse_solver");
		CODEIT_REGISTER_TYPE(DvrkMasterInverseKinematicSolver);
		CODEIT_DECLARE_BIG_FOUR(DvrkMasterInverseKinematicSolver);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};

	class DvrkMasterInverseDynamicSolver :public codeit::model::InverseDynamicSolver
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto virtual allocateMemory()->void override;
		auto virtual dynAccAndFce()->void override;
		auto dynSubRegressorMatrix(const double* pos, const double* vel, const double* acc)->void;
	
		virtual ~DvrkMasterInverseDynamicSolver() = default;
		explicit DvrkMasterInverseDynamicSolver(const std::string& name = "dvrk_master_dynamic_solver");
		CODEIT_REGISTER_TYPE(DvrkMasterInverseDynamicSolver);
		CODEIT_DECLARE_BIG_FOUR(DvrkMasterInverseDynamicSolver);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	///
	/// @}
}

#endif
