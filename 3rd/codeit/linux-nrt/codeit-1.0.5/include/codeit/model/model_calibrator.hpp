#ifndef CODEIT_DYNAMIC_MODEL_CALIBRATOR_H_
#define CODEIT_DYNAMIC_MODEL_CALIBRATOR_H_

#include <codeit/model/model_basic.hpp>

namespace codeit::model
{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	class Calibrator :public Element
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto virtual allocateMemory()->void = 0;
		auto virtual reset()->void = 0;
		auto virtual clb()->int = 0;
		auto error()const->double;
		auto setError(double error)->void;
		auto maxError()const->double;
		auto setMaxError(double max_error)->void;
		auto iterCount()const->Size;
		auto setIterCount(Size iter_count)->void;
		auto maxIterCount()const->Size;
		auto setMaxIterCount(Size max_count)->void;

		virtual ~Calibrator();
		explicit Calibrator(const std::string& name = "calibrator", Size max_iter_count = 100, double max_error = 1e-10);
		CODEIT_REGISTER_TYPE(Calibrator);
		CODEIT_DECLARE_BIG_FOUR(Calibrator);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class ZeroCalibrator : public Calibrator
	{
	public:
		auto virtual reset()->void override;
		auto virtual allocateMemory()->void override;
		auto virtual clb()->int override;
		auto dataImport(const Size, const double)->void;
		virtual ~ZeroCalibrator();
		explicit ZeroCalibrator(const std::string& name = "zero_calibrator", Size max_iter_count = 100, double max_error = 1e-10);
		CODEIT_REGISTER_TYPE(ZeroCalibrator);
		CODEIT_DECLARE_BIG_FOUR(ZeroCalibrator);

	public:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	
	class ToolCalibrator4P : public Calibrator
	{
	public:
		auto virtual reset()->void override;
		auto virtual allocateMemory()->void override;
		auto virtual clb()->int override;
		auto dataImport(const double* pm_set=nullptr)->void;
		auto cal_TCP(const double* transmatric, double tcp[3], double& tcp_error)->int;
		auto tm2RP_4Pt(const double* tm, double* R, double* P)->int;
		auto deltaRP_4Pt(double R[36], double P[12], double* deltaR, double* deltaP)->int;
		
		
		virtual ~ToolCalibrator4P();
		explicit ToolCalibrator4P(const std::string& name = "tool_calibrator4P", Size max_iter_count = 100, double max_error = 1e-10);
		CODEIT_REGISTER_TYPE(ToolCalibrator4P);
		CODEIT_DECLARE_BIG_FOUR(ToolCalibrator4P);

	public:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class ToolCalibrator5P : public Calibrator
	{
	public:
		auto virtual reset()->void override;
		auto virtual allocateMemory()->void override;
		auto virtual clb()->int override;
		auto dataImport(const double* pm_set = nullptr)->void;
		auto cal_TCP_Z(const double* transmatric, double tcp[3], double& tcp_error, double tcf[9])->int;
		auto tm2RP_5Pt(const double* tm, double* R, double* P)->int;
		auto deltaRP_5Pt(double R[45], double P[15], double* deltaR, double* deltaP)->int;
		virtual ~ToolCalibrator5P();
		explicit ToolCalibrator5P(const std::string& name = "tool_calibrator5P", Size max_iter_count = 100, double max_error = 1e-10);
		CODEIT_REGISTER_TYPE(ToolCalibrator5P);
		CODEIT_DECLARE_BIG_FOUR(ToolCalibrator5P);

	public:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class ToolCalibrator6P : public Calibrator
	{
	public:
		auto virtual reset()->void override;
		auto virtual allocateMemory()->void override;
		auto virtual clb()->int override;
		auto dataImport(const double* pm_set_ = nullptr)->void;
		auto cal_TCP_TCF(const double* transmatric, double tcp[3], double& tcp_error, double tcf[9])->int;
		auto tm2RP_6Pt(const double* tm, double* R, double* P)->int;
		auto deltaRP_6Pt(double R[54], double P[18], double* deltaR, double* deltaP)->int;
		virtual ~ToolCalibrator6P();
		explicit ToolCalibrator6P(const std::string& name = "tool_calibrator6P", Size max_iter_count = 100, double max_error = 1e-10);
		CODEIT_REGISTER_TYPE(ToolCalibrator6P);
		CODEIT_DECLARE_BIG_FOUR(ToolCalibrator6P);

	public:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};


	class WobjCalibrator :public Calibrator
	{
	public:
		auto virtual reset()->void override;
		auto virtual allocateMemory()->void override;
		auto virtual clb()->int override;
		virtual ~WobjCalibrator();
		explicit WobjCalibrator(const std::string& name = "wobj_calibrator", Size max_iter_count = 100, double max_error = 1e-10);
		CODEIT_REGISTER_TYPE(WobjCalibrator);
		CODEIT_DECLARE_BIG_FOUR(WobjCalibrator);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	
	
	class KinematicCalibrator :public Calibrator
	{
	public:
		auto virtual reset()->void override;
		auto virtual allocateMemory()->void override;
		auto virtual clb()->int override;
		
		virtual ~KinematicCalibrator();
		explicit KinematicCalibrator(const std::string& name = "kinematic_calibrator", Size max_iter_count = 100, double max_error = 1e-10);
		CODEIT_REGISTER_TYPE(KinematicCalibrator);
		CODEIT_DECLARE_BIG_FOUR(KinematicCalibrator);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class DynamicCalibrator :public Calibrator
	{
	public:
		auto virtual reset()->void override;
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		auto m()->Size;
		auto g()->Size;
		auto k()->Size;
		auto A()->double*;
		auto x()->double*;
		auto b()->double*;
		auto n()->Size { return g() + k(); }
		auto clb0()->void;
		auto dataImport(const Size, std::vector<std::vector<double> >& pos, std::vector<std::vector<double> >& tor)->void;
		auto virtual allocateMemory()->void override;
		auto virtual clb()->int override;
		auto updateInertiaParam(const double* x)->void;
		auto setCutFreq(double freq)->void;
		auto setSamplePrd(double prd)->void;
		virtual ~DynamicCalibrator();
		explicit DynamicCalibrator(const std::string& name = "dynamic_calibrator", Size max_iter_count = 100, double max_error = 1e-10, double cutFreq=5, double samplePrd=0.008);
		CODEIT_REGISTER_TYPE(DynamicCalibrator);
		CODEIT_DECLARE_BIG_FOUR(DynamicCalibrator);
	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	
	class LoadCalibrator :public Calibrator
	{
	public:
		auto virtual reset()->void override;
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		auto m()->Size;
		auto g()->Size;
		auto k()->Size;
		auto A()->double*;
		auto x()->double*;
		auto b()->double*;
		auto n()->Size { return g() + k(); }
		auto clb0()->void;
		auto dataImport(const Size, std::vector<std::vector<double> >& pos, std::vector<std::vector<double> >& tor)->void;
		auto virtual allocateMemory()->void override;
		auto virtual clb()->int override;
		auto updateInertiaParam(const double* x)->void;

		virtual ~LoadCalibrator();
		explicit LoadCalibrator(const std::string& name = "load_calibrator", Size max_iter_count = 100, double max_error = 1e-10, double cutFreq = 5, double samplePrd = 0.008);
		CODEIT_REGISTER_TYPE(LoadCalibrator);
		CODEIT_DECLARE_BIG_FOUR(LoadCalibrator);
	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	///
	/// @}
}

#endif
