#ifndef CODEIT_SPLINE_PLAN_H_
#define CODEIT_SPLINE_PLAN_H_

#include <vector>
#include <cstdint>
using namespace std;
#include <codeit/core/basictype.hpp>
namespace codeit::model
{
	auto s_akima(Size n, const double* x, const double* y, double* p1, double* p2, double* p3, double zero_check = 1e-10)->void;
	auto s_akima_at(Size n, const double* x, const double* y, const double* p1, const double* p2, const double* p3, double x_1, const char order = '0')->double;


	////****************************B Spline Global Interpolate*****************************/////
	Size WhichSpan(double u, vector<double>& U, Size n_knot, Size p);
	void KnotGet(vector<vector<double> >& q, vector<double> &ut,Size column);
	void ControlPtsGet(vector<double>& ut, vector<double> u, vector<vector<double> > q, Size column,Size p, vector<vector<double> > &P);

	void BasisFuns(Size i, double u, Size p, vector<double>& U, vector<double> &B);
	
	void DersBasisFuns(double u, Size i, Size p, int n, vector<double> U, vector<vector<double> > &Ders);
	void quatPtsGet(vector<vector<double> >& teachPts, Size col, vector<double>& quatP_vec);

	void splineInit(vector<vector<double> >& teachPts, Size col,Size p, vector<double>& u, vector<double>& P_vec, vector<double>& quatP_vec,double& arcLength);
	
	
	
	void BSplinePoint(double u, vector<vector<double> >& teachPts, vector<double>& U, Size p, vector<double>& P, vector<double>& quatP_vec,vector<double> &s, vector<double> &ds,vector<double> &dds, double* pm2);

	//void moveSpline(std::int64_t count, vector<vector<double> > teachPts, vector<double> u,int p, vector<double>& P_vec, vector<double>& quatP_vec, double &jMax, double& aMax, double& vMax, const double sMax,double& vlim, double& alim,double* Tt, double& st, double& vt, double& at, double& jt, double ut0, double ut1, double&ut, double&dut, double&ddut, const int count_change, bool& init_flag, bool& vary_flag, bool& finish_flag, double *pm, double &t,double& uSpline, vector<double> &ds0, vector<double> &dds0);

	////****************************Bezier Local Interpolate*****************************/////
	Size WhichSpanBezier(double u, vector<double>& U, double uk_begin, int n_knot);
	void CrossPtsGet(vector<vector<double> >& q, Size column, double delta, vector<vector<double> > &q_line, bool firstMoveLL=false);
	void TCvectorGet(vector<vector<double> >& q_line, vector<vector<double> > &t0, vector<vector<double> > &n0, vector<vector<double> > &t5, vector<vector<double> > &n5);
	void ControlPtsGetBezier(vector<vector<double> >& q_bezier, vector<vector<double> > t0, vector<vector<double> > t5, vector<vector<double> > &P);
	
	void UkGet(vector<vector<double> >& q_line, vector<vector<double> > P, vector<double> &uk, vector<double> &lamda, vector<double>& quatP_vec, double&);
	
	void UkGetLine(vector<vector<double> >& q_line, vector<double>& uk, vector<double>& lamda);
	
	void BezierPoint(double u, double uk_begin,vector<vector<double> >& q_line, vector<vector<double> >& P, vector<double>& uk, vector<double>& lamda, vector<double>& quatP_vec, vector<double>& b, vector<double>& db, vector<double>& ddb, double* pm2, std::int32_t& retCode);

}
#endif