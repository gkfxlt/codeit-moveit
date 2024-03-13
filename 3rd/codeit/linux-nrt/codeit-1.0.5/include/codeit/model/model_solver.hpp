﻿#ifndef CODEIT_DYNAMIC_MODEL_SOLVER_H_
#define CODEIT_DYNAMIC_MODEL_SOLVER_H_

#include <codeit/model/model_basic.hpp>

namespace codeit::model
{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	class Solver :public Element
	{
	public:
		auto virtual saveXml(core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement &xml_ele)->void override;
		auto virtual allocateMemory()->void = 0;
		auto virtual kinPos()->int = 0;
		auto virtual kinVel()->void = 0;
		auto virtual dynAccAndFce()->void = 0;
		auto error()const->double;
		auto setError(double error)->void;
		auto maxError()const->double;
		auto setMaxError(double max_error)->void;
		auto iterCount()const->Size;
		auto setIterCount(Size iter_count)->void;
		auto maxIterCount()const->Size;
		auto setMaxIterCount(Size max_count)->void;

		virtual ~Solver();
		explicit Solver(const std::string &name = "solver", Size max_iter_count = 100, double max_error = 1e-10);
		CODEIT_REGISTER_TYPE(Solver);
		CODEIT_DECLARE_BIG_FOUR(Solver);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class UniversalSolver : public Solver
	{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;
		auto virtual kinVel()->void override;
		auto virtual dynAccAndFce()->void override;
		auto cptGeneralJacobi() noexcept->void;// all_part_vs = Jg * theta_dot, all_part_as = Jg * theta_dot_dot + cg
		auto mJg()const noexcept->Size;// = part_number x 6
		auto nJg()const noexcept->Size;// = motion_number + general_motion_number x 6
		auto Jg()const noexcept->const double *;// dimension : mJ x nJ 
		auto cg()const noexcept->const double *;// dimension : mJ x 1
		auto cptGeneralInverseDynamicMatrix() noexcept->void;// torque = M * theta_dot_dot + h
		auto nM()const noexcept->Size;// = motion_number + general_motion_number x 6
		auto M()const noexcept->const double *;// dimension : nM x nM 
		auto h()const noexcept->const double *;// dimension : nM x 1

		virtual ~UniversalSolver();
		explicit UniversalSolver(const std::string &name = "diag_solver", Size max_iter_count = 100, double max_error = 1e-10);
		CODEIT_REGISTER_TYPE(UniversalSolver);
		CODEIT_DECLARE_BIG_FOUR(UniversalSolver);

	public:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class ForwardKinematicSolver :public UniversalSolver
	{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;
		auto virtual kinVel()->void override;
		auto virtual dynAccAndFce()->void override;
		auto cptJacobi()noexcept->void;
		auto cptJacobiWrtEE(const double* pp_in_base = nullptr)noexcept->void;// wrt ee.makI(), not compute cf
		auto mJf()const noexcept->Size;// equal mot num
		auto nJf()const noexcept->Size;// equal ee num * 6
		auto Jf()const noexcept->const double *;// inverse jacobi   ee_vs = Jf * ee_vs + mot_vs
		auto cf()const noexcept->const double *;// dimension : mJ x 1

		////***新增
		auto cptJacobiWrtPm(const double* wobj_pm = nullptr, const double* target_pm = nullptr)noexcept->void;// wrt ee.makI(), not compute cf


		virtual ~ForwardKinematicSolver();
		explicit ForwardKinematicSolver(const std::string &name = "forward_kinematic_solver", Size max_iter_count = 100, double max_error = 1e-10);
		CODEIT_REGISTER_TYPE(ForwardKinematicSolver);
		CODEIT_DECLARE_BIG_FOUR(ForwardKinematicSolver);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class InverseKinematicSolver :public UniversalSolver
	{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;
		auto virtual kinVel()->void override;
		auto virtual dynAccAndFce()->void override;
		auto cptJacobi()noexcept->void;
		auto mJi()const noexcept->Size;// equal mot num
		auto nJi()const noexcept->Size;// equal ee num * 6
		auto Ji()const noexcept->const double *;// inverse jacobi   mot_vs = Ji * ee_vs
		auto ci()const noexcept->const double *;// dimension : mJ x 1

		virtual ~InverseKinematicSolver();
		explicit InverseKinematicSolver(const std::string &name = "inverse_kinematic_solver", Size max_iter_count = 100, double max_error = 1e-10);
		CODEIT_REGISTER_TYPE(InverseKinematicSolver);
		CODEIT_DECLARE_BIG_FOUR(InverseKinematicSolver);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class ForwardDynamicSolver :public UniversalSolver
	{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;
		auto virtual kinVel()->void override;
		auto virtual dynAccAndFce()->void override;

		virtual ~ForwardDynamicSolver();
		explicit ForwardDynamicSolver(const std::string &name = "forward_dynamic_solver", Size max_iter_count = 100, double max_error = 1e-10);
		CODEIT_REGISTER_TYPE(ForwardDynamicSolver);
		CODEIT_DECLARE_BIG_FOUR(ForwardDynamicSolver);
	};
	class InverseDynamicSolver :public UniversalSolver
	{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;
		auto virtual kinVel()->void override;
		auto virtual dynAccAndFce()->void override;

		virtual ~InverseDynamicSolver();
		explicit InverseDynamicSolver(const std::string &name = "inverse_dynamic_solver", Size max_iter_count = 100, double max_error = 1e-10);
		CODEIT_REGISTER_TYPE(InverseDynamicSolver);
		CODEIT_DECLARE_BIG_FOUR(InverseDynamicSolver);
	};
	///
	/// @}
}

#endif