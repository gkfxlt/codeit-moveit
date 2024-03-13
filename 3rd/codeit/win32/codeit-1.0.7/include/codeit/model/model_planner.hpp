#ifndef CODEIT_DYNAMIC_MODEL_PLANNER_H_
#define CODEIT_DYNAMIC_MODEL_PLANNER_H_

#include <codeit/model/model_basic.hpp>

namespace codeit::model
{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	class Planner :public Element
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto virtual allocateMemory()->void = 0;
		auto virtual reset()->void = 0;
		
		auto isIdle()->bool;
		auto setIdle(bool)->void;

		virtual ~Planner();
		explicit Planner(const std::string& name = "planner");
		CODEIT_REGISTER_TYPE(Planner);
		CODEIT_DECLARE_BIG_FOUR(Planner);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class JointPlanner : public Planner
	{
	public:
		auto virtual allocateMemory()->void;
		auto virtual reset()->void override;
		auto virtual planning(const Size id, const Size, const vector<double>, const std::int64_t, \
			vector<double>&, vector<double>& acc, vector<double>&, \
			vector<double>&, vector<double>&, vector<double>&, vector<double>&, vector<double>&, \
			vector<double>&, vector<double>&, vector<double>& vlim, vector<double>& alim, \
			vector<int>& init_flag, vector<int>& running_flag, std::int32_t& retCode)->bool;

		
		virtual ~JointPlanner();
		explicit JointPlanner(const std::string& name = "joint_planner", Size max_iter_count = 100, double max_error = 1e-10);
		CODEIT_REGISTER_TYPE(JointPlanner);
		CODEIT_DECLARE_BIG_FOUR(JointPlanner);

	public:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	
	class MoveLPlanner : public Planner
	{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual reset()->void override;
		auto virtual planning(const Size id, const std::int64_t count, const double* begin_pm, const double* end_pm,\
			double& jerk, double& acc, double& dec, double& vel, double& angular_jerk, \
			double& angular_acc, double& angular_dec, double& angular_vel, \
			double* Tt, double* st, double* vt, double* at, double* jt, \
			double* pm2, double* t, std::int32_t& retCode)->bool;

		virtual ~MoveLPlanner();
		explicit MoveLPlanner(const std::string& name = "movel_planner", Size max_iter_count = 100, double max_error = 1e-10);
		CODEIT_REGISTER_TYPE(MoveLPlanner);
		CODEIT_DECLARE_BIG_FOUR(MoveLPlanner);

	public:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};

	class MoveCPlanner : public Planner
	{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual reset()->void override;
		auto virtual planning(const Size id, const std::int64_t count, const double* begin_pq, const double* mid_pq, const double* end_pq, \
			double& jerk, double& acc, double& dec, double& vel, double& angular_jerk, \
			double& angular_acc, double& angular_dec, double& angular_vel, \
			double* Tt, double* st, double* vt, double* at, double* jt, \
			double* pm2, double* t, double* C, double& R, double* axis, double& theta, std::int32_t& retCode)->bool;

		virtual ~MoveCPlanner();
		explicit MoveCPlanner(const std::string& name = "movec_planner", Size max_iter_count = 100, double max_error = 1e-10);
		CODEIT_REGISTER_TYPE(MoveCPlanner);
		CODEIT_DECLARE_BIG_FOUR(MoveCPlanner);

	public:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	
	class MoveSPlanner : public Planner
	{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual reset()->void override;
		auto virtual planning(const Size id, const std::int64_t count, \
			vector<vector<double> > teachPts, vector<double> u, int p, vector<double>& P_vec, \
			vector<double>& quatP_vec, double& jMax, double& aMax, double& aMin, double& vMax, const double sMax, \
			double* Tt, double& st, double& vt, double& at, double& jt, \
			double* pm, double& t, double& uSpline, vector<double>& ds0, vector<double>& dds0, std::int32_t retCode)->bool;

		virtual ~MoveSPlanner();
		explicit MoveSPlanner(const std::string& name = "moves_planner", Size max_iter_count = 100, double max_error = 1e-10);
		CODEIT_REGISTER_TYPE(MoveSPlanner);
		CODEIT_DECLARE_BIG_FOUR(MoveSPlanner);

	public:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};

	class MoveLLPlanner : public Planner
	{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual reset()->void override;
		auto virtual planning(const Size id, const std::int64_t count, \
			vector<vector<double> > teachPts, int p, vector<vector<double>>& P_vec, \
			vector<vector<double> >& ,vector<double>& quatP_vec, vector<double>& uk, vector<double>& lamda, \
			double& jMax, double& jMin, double& aMax, double& aMin, double& vMax, const double sMax, \
			double* Tt, double& st, double& vt, double& at, double& jt, \
			double* pm, double& t, const double dt, double& uBezier, vector<double>& db, vector<double>& ddb, std::int32_t& retCode, const bool)->bool;

		virtual ~MoveLLPlanner();
		explicit MoveLLPlanner(const std::string& name = "movell_planner", Size max_iter_count = 100, double max_error = 1e-10);
		CODEIT_REGISTER_TYPE(MoveLLPlanner);
		CODEIT_DECLARE_BIG_FOUR(MoveLLPlanner);

	public:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};

	class ServoJPlanner : public Planner
	{
	public:
		double sign(double x);
		double sat(double x);
		auto virtual allocateMemory()->void override;
		auto virtual reset()->void override;
		auto virtual planning(const Size id, const std::int64_t count, \
			const Size num, const vector<double>& aMax, const vector<double>& vMax, \
			const vector<double>& ,const vector<double>& r, vector<double>& q, vector<double>& dq, \
			const double time, const double t, const double dt, const bool endServoJ, std::int32_t& retCode)->bool;

		virtual ~ServoJPlanner();
		explicit ServoJPlanner(const std::string& name = "servoj_planner", Size max_iter_count = 100, double max_error = 1e-10);
		CODEIT_REGISTER_TYPE(ServoJPlanner);
		CODEIT_DECLARE_BIG_FOUR(ServoJPlanner);

	public:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};


	class ServoJPlanner2 : public Planner
	{
	public:
		double sign(double x);
		double sat(double x);
		auto virtual allocateMemory()->void override;
		auto virtual reset()->void override;
		auto virtual planning(const Size id, const std::int64_t count, \
			const Size num, const vector<double>& aMax, const vector<double>& vMax, \
			const vector<double>&, const vector<double>& r, vector<double>& q, vector<double>& dq, \
			const double time, const double t, const double dt, const bool endServoJ, std::int32_t& retCode)->bool;

		virtual ~ServoJPlanner2();
		explicit ServoJPlanner2(const std::string& name = "servoj_planner2", Size max_iter_count = 100, double max_error = 1e-10);
		CODEIT_REGISTER_TYPE(ServoJPlanner2);
		CODEIT_DECLARE_BIG_FOUR(ServoJPlanner2);

	public:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	///
	/// @}


	class EGMPlanner : public Planner
	{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual reset()->void override;
		auto virtual planning(const Size id, const std::int64_t count, \
			const double* dX, const double* gain, const bool frame, const double dt, double* pm_now, \
			double* pm2, std::int32_t& retCode)->bool;

		virtual ~EGMPlanner();
		explicit EGMPlanner(const std::string& name = "egm_planner", const double cutoff_freq = 0);

		CODEIT_REGISTER_TYPE(EGMPlanner);
		CODEIT_DECLARE_BIG_FOUR(EGMPlanner);

	public:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
}



#endif
