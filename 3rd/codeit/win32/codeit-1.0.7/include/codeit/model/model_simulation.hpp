#ifndef CODEIT_MODEL_MODEL_SIMULATION_H_
#define CODEIT_MODEL_MODEL_SIMULATION_H_

#include <codeit/model/model_interaction.hpp>
#include <codeit/model/model_solver.hpp>


namespace codeit::function
{
	//struct PlanTarget;
	class BasisFunc;
}

namespace codeit::model
{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///

	class SimResult : public Element
	{
	public:
		class TimeResult : public Element
		{
		public:
			auto virtual saveXml(core::XmlElement &xml_ele) const->void override;
			auto virtual loadXml(const core::XmlElement &xml_ele)->void override;
			auto record()->void;
			auto restore(Size pos)->void;

			virtual ~TimeResult();
			explicit TimeResult(const std::string &name = "time_result");
			CODEIT_REGISTER_TYPE(TimeResult);
			CODEIT_DECLARE_BIG_FOUR(TimeResult);

		private:
			struct Imp;
			core::ImpPtr<Imp> imp_;

			friend class SimResult;
		};
		class PartResult : public Element
		{
		public:
			auto virtual saveXml(core::XmlElement &xml_ele) const->void override;
			auto virtual loadXml(const core::XmlElement &xml_ele)->void override;
			auto part()->Part&;
			auto part()const->const Part& { return const_cast<PartResult*>(this)->part(); }
			auto record()->void;
			auto restore(Size pos)->void;

			virtual ~PartResult();
			explicit PartResult(const std::string &name = "part_result", Part *part = nullptr);
			CODEIT_REGISTER_TYPE(PartResult);
			CODEIT_DECLARE_BIG_FOUR(PartResult);

		private:
			struct Imp;
			core::ImpPtr<Imp> imp_;

			friend class SimResult;
		};
		class ConstraintResult : public Element
		{
		public:
			auto virtual saveXml(core::XmlElement &xml_ele) const->void override;
			auto virtual loadXml(const core::XmlElement &xml_ele)->void override;
			auto constraint()->Constraint&;
			auto constraint()const->const Constraint& { return const_cast<ConstraintResult*>(this)->constraint(); }
			auto record()->void;
			auto restore(Size pos)->void;

			virtual ~ConstraintResult();
			explicit ConstraintResult(const std::string &name = "constraint_result", Constraint *constraint = nullptr);
			CODEIT_REGISTER_TYPE(ConstraintResult);
			CODEIT_DECLARE_BIG_FOUR(ConstraintResult);

		private:
			struct Imp;
			core::ImpPtr<Imp> imp_;

			friend class SimResult;
		};

		auto virtual loadXml(const core::XmlElement &xml_ele)->void override;
		auto timeResult()->TimeResult&;
		auto timeResult()const->const TimeResult& { return const_cast<SimResult*>(this)->timeResult(); }
		auto partResultPool()->core::ObjectPool<PartResult, Element>&;
		auto partResultPool()const->const core::ObjectPool<PartResult, Element>& { return const_cast<SimResult*>(this)->partResultPool(); };
		auto constraintResultPool()->core::ObjectPool<ConstraintResult, Element>&;
		auto constraintResultPool()const->const core::ObjectPool<ConstraintResult, Element>& { return const_cast<SimResult*>(this)->constraintResultPool(); };

		auto allocateMemory()->void;
		auto record()->void;
		auto restore(Size pos)->void;
		auto size()const->Size;
		auto clear()->void;

		virtual ~SimResult();
		explicit SimResult(const std::string &name = "sim_result");
		CODEIT_REGISTER_TYPE(SimResult);
		CODEIT_DECLARE_BIG_FOUR(SimResult);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class Simulator :public Element
	{
	public:
		auto virtual simulate(codeit::function::BasisFunc &plan, SimResult &result)->void;

		virtual ~Simulator();
		explicit Simulator(const std::string &name = "simulator");
		CODEIT_REGISTER_TYPE(Simulator);
		CODEIT_DECLARE_BIG_FOUR(Simulator);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class SolverSimulator : public Simulator
	{
	public:
		auto virtual saveXml(core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement &xml_ele)->void override;
		auto virtual simulate(codeit::function::BasisFunc &plan, SimResult &result)->void override;
		using Simulator::simulate;
		auto solver()->Solver&;
		auto solver()const ->const Solver& { return const_cast<SolverSimulator*>(this)->solver(); };

		virtual ~SolverSimulator();
		explicit SolverSimulator(const std::string &name = "solver_simulator", Solver *solver = nullptr);
		CODEIT_REGISTER_TYPE(SolverSimulator);
		CODEIT_DECLARE_BIG_FOUR(SolverSimulator);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class AdamsSimulator :public Simulator
	{
	public:
		auto saveAdams(const std::string &filename, SimResult &result, Size pos = -1)->void;
		auto saveAdams(std::ofstream &file, SimResult &result, Size pos = -1)->void;
		auto saveAdams(const std::string &filename)->void;
		auto saveAdams(std::ofstream &file)->void;
		auto adamsID(const Marker &mak)const->Size;
		auto adamsID(const Part &prt)const->Size;
		auto adamsID(const Element &ele)const->Size { return ele.id() + 1; };

		virtual ~AdamsSimulator();
		explicit AdamsSimulator(const std::string &name = "adams_solver");
		CODEIT_REGISTER_TYPE(AdamsSimulator);
		CODEIT_DECLARE_BIG_FOUR(AdamsSimulator);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};

	// 辨识会得到 A 、 x 、 b 这样的矩阵和向量
	// 理论上 A * x = b
	// A为 m * n 维，x 为 n * 1 维，b维 m * 1维
	// 
	// m 为电机个数，也就是当前点的方程数，比如这里就是6
	// n 为待辨识的参数，它为杆件数(不含地面) * 10 + 电机 * 3，因此这里是78
	//
	// 杆件的辨识参数如下，其中xyz为质心位置：
	// m m*x m*y m*z Ixx Iyy Izz Ixy Ixz Iyz
	// 电机的辨识参数如下，也就是静摩擦力、粘性摩擦系数、电机转子惯量：
	// fs kv ki
	// 其中电机摩擦力计算为： 
	// f = sig(v)*fs + kv * v + ki * a
	//
	// x为当前的惯量值，注意它并不是辨识出来的结果，它仅仅保存了当前model中各个杆件的惯量和电机参数
	// b为当前的电机出力
	// A为观测矩阵
	class CalibratorY :public Element
	{
	public:
		auto virtual allocateMemory()->void;
		auto m()->Size;
		auto n()->Size { return g() + k(); }
		auto g()->Size;
		auto k()->Size;
		auto A()->double*;
		auto x()->double*;
		auto b()->double*;
		auto clb()->void;
		auto clbFile(const std::string &file_path)->void;
		auto clbFiles(const std::vector<std::string> &file_paths)->void;
		auto verifyFiles(const std::vector<std::string> &file_paths)->void;
		auto updateInertiaParam(const double *inertia_param)->void;

		virtual ~CalibratorY();
		explicit CalibratorY(const std::string &name = "calibratorY");
		CODEIT_REGISTER_TYPE(CalibratorY);
		CODEIT_DECLARE_BIG_FOUR(CalibratorY);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};

	/// @}
}

#endif
