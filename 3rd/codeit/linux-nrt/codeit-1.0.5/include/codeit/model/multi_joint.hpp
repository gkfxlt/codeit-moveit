#ifndef CODEIT_DYNAMIC_MULTI_JOINT_H_
#define CODEIT_DYNAMIC_MULTI_JOINT_H_

#include <codeit/model/model_solver.hpp>

namespace codeit::model
{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	struct MultiJointParam
	{
		// DH PARAM //
		Size dof{ 0 };

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

		// 驱动在零位处的偏移，以及系数
		double mp_offset[3];// mp_real = (mp_theoretical - mp_offset) * mp_factor
		double mp_factor[3];
	};
	auto createModelMultiJoint(const MultiJointParam& param, std::string name = "model")->std::unique_ptr<codeit::model::Model>;
	
	///
	/// @}
}

#endif
