#ifndef CODEIT_CORE_FILTER_H_
#define CODEIT_CORE_FILTER_H_
#include <codeit/core/object.hpp>
namespace codeit::core
{
	//截至频率为角频率=2*pi*fc(频率)//
	class LPF1st:public Object{
	public:
		auto virtual allocateMemory()->void;
		auto virtual reset()->void;
		auto virtual processing(const double raw, const double dt, double* processed)->void;
		auto virtual cutoffFreq()->double;
		auto virtual setCutoffFreq(double)->void;
		virtual ~LPF1st();
		explicit LPF1st(const double cutoff_freq = 0);
		CODEIT_REGISTER_TYPE(LPF1st);
		CODEIT_DECLARE_BIG_FOUR(LPF1st);


	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	//截至频率为角频率=2*pi*fc(频率)//
	class LPF2nd :public Object {
	public:
		auto virtual allocateMemory()->void;
		auto virtual reset()->void;
		auto virtual processing(const double raw, const double dt, double* processed)->void;
		auto virtual cutoffFreq()->double;
		auto virtual setCutoffFreq(double)->void; virtual ~LPF2nd();
		explicit LPF2nd(const double cutoff_freq = 0);
		CODEIT_REGISTER_TYPE(LPF2nd);
		CODEIT_DECLARE_BIG_FOUR(LPF2nd);
	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};

	//截至频率为角频率=2*pi*fc(频率)//
	class LPF3rd :public Object {
	public:
		auto virtual allocateMemory()->void;
		auto virtual reset()->void;
		auto virtual processing(const double raw, const double dt, double* processed)->void;
		auto virtual cutoffFreq()->double;
		auto virtual setCutoffFreq(double)->void; 
		virtual ~LPF3rd();
		explicit LPF3rd(const double cutoff_freq = 0);
		CODEIT_REGISTER_TYPE(LPF3rd);
		CODEIT_DECLARE_BIG_FOUR(LPF3rd);
	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	

	//截至频率为频率//
	/******测试陷波器*****/
	/*
	double Wo = 60, Q = 35;
	double dt = 0.001;
	Notch2nd filter(dt, Wo, Q);
	double x, y;
	for (int i = 0; i < 1000; i++)
	{
		x = sin(dt*i);
		filter.processing(x, y);
	}
	*/

	class Notch2nd :public Object {
	public:
		auto virtual allocateMemory()->void;
		auto virtual reset()->void;
		auto virtual processing(const double raw, double& processed)->void;
		auto virtual notchFreq()->double;
		auto virtual setNotchFreq(double)->void;
		auto virtual qfactor()->double;
		auto virtual setQfactor(double)->void;
		auto virtual sampleFreq()->double;
		auto virtual setSampleFreq(double)->void;
		virtual ~Notch2nd();
		explicit Notch2nd(const double dt=0.001, const double notc_freq = 0, const double qfactor=35);
		CODEIT_REGISTER_TYPE(Notch2nd);
		CODEIT_DECLARE_BIG_FOUR(Notch2nd);
	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};

}


#endif