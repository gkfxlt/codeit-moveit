#ifndef CODEIT_S_Curve_H_
#define CODEIT_S_Curve_H_
#include<codeit/model/matrix.hpp>
#include<codeit/core/basictype.hpp>
#include<codeit/model/screw.hpp>

using namespace codeit::model;
namespace codeit::model
{
	auto inline acc_up(int n, int i)noexcept->double { return (-1.0 / 2 / n / n / n * i*i*i + 3.0 / 2.0 / n / n * i*i); }
	auto inline acc_down(int n, int i)noexcept->double { return (-1.0*i*i*i / 2.0 / n / n / n + 3.0 * i*i / 2.0 / n / n); }
	auto inline dec_up(int n, int i)noexcept->double { return 1.0 - (-1.0 / 2.0 / n / n / n * (n - i)*(n - i)*(n - i) + 3.0 / 2.0 / n / n * (n - i)*(n - i)); }
	auto inline dec_down(int n, int i)noexcept->double { return 1.0 - (-1.0*(n - i)*(n - i)*(n - i) / 2.0 / n / n / n + 3.0 * (n - i)*(n - i) / 2.0 / n / n); }

	auto inline acc_even(int n, int i)noexcept->double { return 1.0 / n / n * i * i; }
	auto inline dec_even(int n, int i)noexcept->double { return 1.0 - 1.0 / n / n * (n - i)*(n - i); }
	auto inline even(int n, int i)noexcept->double { return 1.0 / n * i; }

	auto inline s_p2p(int n, int i, double begin_pos, double end_pos)noexcept->double
	{
		double a = 4.0 * (end_pos - begin_pos) / n / n;
		return i <= n / 2.0 ? 0.5*a*i*i + begin_pos : end_pos - 0.5*a*(n - i)*(n - i);
	}
	auto inline s_v2v(int n, int i, double begin_vel, double end_vel)noexcept->double
	{
		double s = static_cast<double>(i) / n;
		double m = 1.0 - s;

		return (s*s*s - s * s)*end_vel*n + (m*m - m * m*m)*begin_vel*n;
	}
	auto inline s_interp(int n, int i, double begin_pos, double end_pos, double begin_vel, double end_vel)noexcept->double
	{
		double s = static_cast<double>(i) / n;

		double a, b, c, d;

		c = begin_vel * n;
		d = begin_pos;
		a = end_vel * n - 2.0 * end_pos + c + 2.0 * d;
		b = end_pos - c - d - a;

		return a * s*s*s + b * s*s + c * s + d;
	}

	auto moveAbsolute(double i, double begin_pos, double end_pos, double vel, double acc, double dec, double &current_pos, double &current_vel, double &current_acc, Size& total_count)->void;
	auto moveAbsolute2(double pa, double va, double aa, double pt, double vt, double at, double vm, double am, double dm, double dt, double zero_check, double &pc, double &vc, double &ac, Size& total_count)->int;






	auto ratedAdj(const Size id, const std::int64_t count, const std::int64_t count_change, const double u0, const double u1, double &u, double &du, double &ddu)->void;


	auto sCurVelParamGet(double jMax, double aMax, double vMax, double sMax, double *T, double& vlim, double& alim)->void;
	auto sCurVelParamNorm(double& jMax, double& aMax, double& vMax, const double sMax, double *T, double& vlim, double& alim)->void;
	auto sCurVelPro(const Size id, const std::int64_t count, double& jMax, double& aMax, double& vMax, const double sMax, double& vlim, double& alim,\
		double  *Tt, double &st, double &vt, double &at, double &jt, \
		int& init_flag, double &t)->bool;

	auto sCurVelGenParamGet(double& jMax, double& aMax, double& vMax, const double sMax,\
		double *T, double& vlim, double& alima, double& alimd, const double v0, const double v1)->void;
	auto sCurVelProGen(const Size id, const std::int64_t count, double& jMax, double& jMin, double& aMax, double& aMin, double& vMax, double& vMin, \
		const double sMax, double& vlim, double& alima, double& alimd, double* Tt, double& st, double& vt, \
		double& at, double& jt, int& init_flag, double& t, double v0, double v1)->bool;

	auto sCurVelOnlinePro(const Size id, const std::int64_t count, const double jMax, const double jMin, \
		const double aMax, const double aMin, const double vMax, const double vMin, \
		const double sMax, double& st, double& vt, double& at, double& jt, double& t, \
		const double v0, const double v1, const double a0, const double a1, const double dt,
		bool& initHigherVmax, bool& isConstAcc, bool& isStopping, bool& isAccStopping, bool& flag, \
		double& timeRecord, double& Tj2a, double& Tj2b, double& Td, double& h)->bool;




	auto sCurVelSyncMoveJ(const Size NumOfAxis, vector<double>& jMax, \
		vector<double>& aMax, vector<double>& vMax, const vector<double> sMax, \
		vector<double>& vlim, vector<double>& alim, vector<double>& Tt)->void;

	auto sCurVelSyncMoveLaxis(const double* begin_pm, const double *end_pm, double& vel, double& acc, double& jerk, double& angular_vel, double& angular_acc, double& angular_jerk, double* vlim, double* alim, double* Tt)->void;
	auto sCurVelSyncMoveLquat(const double* begin_pq, const double *end_pq, double& vel, double& acc, double& jerk, double& angular_vel, double& angular_acc, double& angular_jerk, double* vlim, double* alim)->void;

	auto sCurVelSyncMoveCaxis(const double* begin_pq, const double *end_pq, double& vel, double& acc, double& jerk, double& angular_vel, double& angular_acc, double& angular_jerk, double* vlim, double* alim, double &R, double &theta)->void;


}

#endif
