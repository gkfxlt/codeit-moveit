#ifndef CODEIT_DYNAMIC_SCARA_H_
#define CODEIT_DYNAMIC_SCARA_H_

#include <array>
#include <codeit/model/model_solver.hpp>

namespace codeit::model
{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	struct ScaraParam
	{
		// DH PARAM //
		double d3{ 0.0 };
		double a2{ 0.0 };
		double a3{ 0.0 };
		

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
	auto createModelScara(const ScaraParam& param, std::string name = "model")->std::unique_ptr<codeit::model::Model>;

	
	///
	/// @}
}

#endif
