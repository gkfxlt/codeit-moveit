#ifndef CODEIT_DYNAMIC_STEWART_H_
#define CODEIT_DYNAMIC_STEWART_H_

#include <codeit/model/model_solver.hpp>

namespace codeit::model
{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///

	auto createModelStewart()->std::unique_ptr<codeit::model::Model>;

	class StewartInverseKinematicSolver :public codeit::model::InverseKinematicSolver
	{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;

		virtual ~StewartInverseKinematicSolver() = default;
		explicit StewartInverseKinematicSolver(const std::string &name = "stewart_inverse_solver");
		CODEIT_REGISTER_TYPE(StewartInverseKinematicSolver);
		CODEIT_DECLARE_BIG_FOUR(StewartInverseKinematicSolver);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	///
	/// @}
}

#endif
