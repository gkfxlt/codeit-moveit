#ifndef CODEIT_DYNAMIC_MODEL_INTERACTION_H_
#define CODEIT_DYNAMIC_MODEL_INTERACTION_H_

#include <cmath>

#include <codeit/model/model_coordinate.hpp>

namespace codeit::model
{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///

	class Interaction :public DynEle
	{
	public:
		auto virtual saveXml(core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement &xml_ele)->void override;
		auto makI() noexcept->Marker& { return *makI_; }
		auto makI() const noexcept->const Marker& { return *makI_; }
		auto makJ() noexcept->Marker& { return *makJ_; }
		auto makJ() const noexcept->const Marker& { return *makJ_; }

		virtual ~Interaction() = default;
		explicit Interaction(const std::string &name = "interaction", Marker *makI = nullptr, Marker *makJ = nullptr, bool is_active = true) :DynEle(name, is_active), makI_(makI), makJ_(makJ) {}
		CODEIT_REGISTER_TYPE(Interaction);
		CODEIT_DEFINE_BIG_FOUR(Interaction);

	private:
		Marker * makI_;
		Marker *makJ_;
	};
	class Constraint :public Interaction
	{
	public:
		auto virtual dim() const->Size = 0;
		auto virtual locCmI() const->const double* = 0;
		auto virtual saveXml(core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement &xml_ele)->void override;
		auto virtual cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const noexcept->void;
		auto virtual cptCp(double *cp)const noexcept->void { cptCpFromPm(cp, *makI().pm(), *makJ().pm()); }
		auto virtual cptCv(double *cv)const noexcept->void;
		auto virtual cptCa(double *ca)const noexcept->void;
		auto virtual cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void
		{
			double cmI[36], cmJ[36];
			double U[36], tau[6];
			double Q[36], R[36];

			cptGlbCmFromPm(cmI, cmJ, makI_pm, makJ_pm);

			s_householder_ut(6, dim(), cmI, U, tau);
			s_householder_ut2qr(6, dim(), U, tau, Q, R);

			double tem[36];
			s_fill(6, 6, 0.0, tem);
			s_fill(6, 1, 1.0, tem, 7);
			s_inv_um(dim(), R, dim(), tem, 6);
			s_mm(6, 6, 6, tem, 6, Q, codeit::model::ColMajor{ 6 }, dm, 6);
		}
		auto virtual cptGlbCmFromPm(double *cmI, double *cmJ, const double *makI_pm, const double *makJ_pm)const noexcept->void
		{
			s_tf_n(dim(), makI_pm, locCmI(), cmI);
			s_mi(6, dim(), cmI, cmJ);
		}
		//template<typename CMI_TYPE, typename CMJ_TYPE>
		//auto virtual cptGlbCmFromPm(double *cmI, double *cmJ, const double *makI_pm, const double *makJ_pm)const noexcept->void
		template<typename CMI_TYPE, typename CMJ_TYPE>
		auto cptCm(const Coordinate &relative_to_I, double *cmI, CMI_TYPE cmi_type, const Coordinate &relative_to_J, double *cmJ, CMJ_TYPE cmj_type)const noexcept->void
		{
			double pm[16];
			makI().getPm(relative_to_I, pm);
			s_tf_n(dim(), pm, locCmI(), dim(), cmI, cmi_type);

			makI().getPm(relative_to_J, pm);
			s_tf_n(dim(), -1.0, pm, locCmI(), dim(), cmJ, cmj_type);
		}
		template<typename CMI_TYPE, typename CMJ_TYPE>
		auto cptCm(const Coordinate &relative_to, double *cmI, CMI_TYPE cmi_type, double *cmJ, CMJ_TYPE cmj_type)const noexcept->void
		{
			double pm[16];
			makI().getPm(relative_to, pm);
			s_tf_n(dim(), pm, locCmI(), dim(), cmI, cmi_type);

			s_mi(6, dim(), cmI, cmi_type, cmJ, cmj_type);
		}
		template<typename CMI_TYPE, typename CMJ_TYPE>
		auto cptPrtCm(double *cmI, CMI_TYPE cmi_type, double *cmJ, CMJ_TYPE cmj_type)const noexcept->void
		{
			cptCm(makI().fatherPart(), cmI, cmi_type, makJ().fatherPart(), cmJ, cmj_type);
		}
		auto cptPrtCm(double *cmI, double *cmJ)->void { cptPrtCm(cmI, dim(), cmJ, dim()); }
		template<typename CMI_TYPE, typename CMJ_TYPE>
		auto cptGlbCm(double *cmI, CMI_TYPE cmi_type, double *cmJ, CMJ_TYPE cmj_type)const noexcept->void
		{
			s_tf_n(dim(), *makI().pm(), locCmI(), dim(), cmI, cmi_type);
			s_mi(6, dim(), cmI, cmi_type, cmJ, cmj_type);
		}
		auto cptGlbCm(double *cmI, double *cmJ)const noexcept->void{ cptGlbCm(cmI, dim(), cmJ, dim()); }
		auto cf() const noexcept->const double*;
		auto setCf(const double *cf) noexcept->void;


		virtual ~Constraint();
		explicit Constraint(const std::string &name = "constraint", Marker *makI = nullptr, Marker *makJ = nullptr, bool is_active = true);
		CODEIT_REGISTER_TYPE(Constraint);
		CODEIT_DECLARE_BIG_FOUR(Constraint);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
		friend class Motion;
		friend class GeneralMotion;
		friend class PointMotion;
	};
	class Joint :public Constraint
	{
	public:
		virtual ~Joint() = default;
		explicit Joint(const std::string &name = "joint", Marker *makI = nullptr, Marker *makJ = nullptr, bool active = true) : Constraint(name, makI, makJ, active) {}
		CODEIT_REGISTER_TYPE(Joint);
		CODEIT_DEFINE_BIG_FOUR(Joint);
	};
	class Motion final :public Constraint
	{
	public:
		static auto Dim()->Size { return 1; }
		auto pSize()const noexcept->Size { return dim(); }
		auto virtual saveXml(core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement &xml_ele)->void override;
		auto virtual dim() const noexcept ->Size override { return Dim(); }
		auto virtual locCmI() const noexcept->const double* override;
		auto virtual cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const noexcept->void override;
		auto virtual cptCv(double *cv)const noexcept->void override;
		auto virtual cptCa(double *ca)const noexcept->void override;
		auto motorId()const->int;
		auto setMotorId(int id)->void;
		auto setAxis(Size axis)->void;

		auto axis()const noexcept->Size;
		auto mp() const noexcept->double;
		auto updMp() noexcept->void;
		auto setMp(double mp) noexcept->void;
		auto mv() const noexcept->double;
		auto updMv() noexcept->void;
		auto setMv(double mv) noexcept->void;
		auto ma() const noexcept->double;
		auto updMa() noexcept->void;
		auto setMa(double ma) noexcept->void;
		auto mf() const noexcept->double;
		auto setMf(double mf) noexcept->void;
		auto mfDyn() const noexcept->double;
		auto setMfDyn(double mf_dyn) noexcept->void;
		auto mfFrc() const noexcept->double;
		auto frcCoe() const noexcept ->const double3&;
		auto setFrcCoe(const double *frc_coe) noexcept->void;
		auto frcZeroCheck()const noexcept ->double;
		auto setFrcZeroCheck(double zero_check)noexcept ->void;
		auto mpOffset()const noexcept->double;
		auto setMpOffset(double mp_offset)noexcept->void;
		auto mpFactor()const noexcept->double;
		auto setMpFactor(double mp_factor)noexcept->void;
		auto mpInternal()const noexcept->double;
		auto setMpInternal(double mp_internal)noexcept->void;

		virtual ~Motion();
		explicit Motion(const std::string &name = "motion", Marker *makI = nullptr, Marker *makJ = nullptr, Size component_axis = 2, const double *frc_coe = nullptr, double mp_offset = 0.0, double mp_factor = 1.0, double zero_check=1.0e-3, bool active = true);
		CODEIT_REGISTER_TYPE(Motion);
		CODEIT_DECLARE_BIG_FOUR(Motion);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};

	class CartesianMotion :public Constraint {
	public:
		auto virtual pSize()const noexcept->Size { return dim(); }
		auto virtual mpm()const noexcept->const double4x4& = 0;
		auto virtual updMpm() noexcept->void {}
		auto virtual setMpm(const double *p) noexcept->void {}
		auto virtual getMpm(double *p)const noexcept->void { s_vc(pSize(), *this->mpm(), p); }
		auto virtual mvs()const noexcept->const double6& = 0;
		auto virtual updMvs() noexcept->void {}
		auto virtual setMvs(const double *v) noexcept->void {}
		auto virtual getMvs(double *v)const noexcept->void { s_vc(dim(), this->mvs(), v); }
		auto virtual mas()const noexcept->const double6& = 0;
		auto virtual updMas() noexcept->void {}
		auto virtual setMas(const double *a) noexcept->void {}
		auto virtual getMas(double *a)const noexcept->void { s_vc(dim(), this->mas(), a); }
		auto virtual mfs()const noexcept->const double6& = 0;
		auto virtual setMfs(const double *f) noexcept->void {}
		auto virtual getMfs(double *f)const noexcept->void { s_vc(dim(), this->mfs(), f); }

		auto virtual setMpe(const double* pe, const char *type = "313") noexcept->void {}
		auto virtual setMpq(const double* pq) noexcept->void {}
		auto virtual getMpe(double* pe, const char *type = "313")const noexcept->void {}
		auto virtual getMpq(double* pq)const noexcept->void {}
		auto virtual setMvq(const double* vq) noexcept->void {}
		auto virtual setMve(const double* ve, const char *type = "313") noexcept->void {}
		auto virtual setMvm(const double* vm) noexcept->void {}
		auto virtual setMva(const double* va) noexcept->void {}
		auto virtual getMve(double* ve, const char *type = "313")const noexcept->void {}
		auto virtual getMvq(double* vq)const noexcept->void {}
		auto virtual getMvm(double* vm)const noexcept->void {}
		auto virtual getMva(double* va)const noexcept->void {}
		auto virtual setMae(const double* ae, const char *type = "313") noexcept->void {}
		auto virtual setMaq(const double* aq) noexcept->void {}
		auto virtual setMam(const double* am) noexcept->void {}
		auto virtual setMaa(const double* aa) noexcept->void {}
		auto virtual getMae(double* ae, const char *type = "313")const noexcept->void {}
		auto virtual getMaq(double* aq)const noexcept->void {}
		auto virtual getMam(double* am)const noexcept->void {}
		auto virtual getMaa(double* aa)const noexcept->void {}

		///***新增
		auto virtual getMpmWrtToolWobj(const double* tool_pm, const double* wobj_pm, double* pm)const noexcept->void {}
		auto virtual setMpmWrtToolWobj(const double* tool_pm, const double* wobj_pm, const double* pm) noexcept->void {}
		virtual ~CartesianMotion() = default;
		explicit CartesianMotion(const std::string &name = "cartesian_Motion", Marker *makI = nullptr, Marker *makJ = nullptr, bool active = true)
			: Constraint(name, makI, makJ, active) {}
		CODEIT_REGISTER_TYPE(CartesianMotion);
		CODEIT_DECLARE_BIG_FOUR(CartesianMotion);
	};


	class GeneralMotion final :public CartesianMotion
	{
	public:
		static auto Dim()->Size { return 6; }
		auto virtual dim() const noexcept ->Size override { return Dim(); }
		auto virtual locCmI() const noexcept->const double* override;
		auto virtual cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const noexcept->void override;
		auto virtual cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void override
		{
			double pm[16];
			s_inv_pm(makI_pm, pm);
			s_tmf(pm, dm);
		}
		auto virtual cptCv(double *cv)const noexcept->void override;
		auto virtual cptCa(double *ca)const noexcept->void override;
		auto virtual pSize()const noexcept->Size { return 16; }

		auto mpm()const noexcept->const double4x4& override;
		auto updMpm() noexcept->void override;
		auto setMpe(const double* pe, const char *type = "313") noexcept->void override;
		auto setMpq(const double* pq) noexcept->void override;
		auto setMpm(const double* pm) noexcept->void override;
		auto getMpe(double* pe, const char *type = "313")const noexcept->void override;
		auto getMpq(double* pq)const noexcept->void override;
		auto getMpm(double* pm)const noexcept->void override;
		auto mvs()const noexcept->const double6& override;
		auto updMvs() noexcept->void override;
		auto setMve(const double* ve, const char *type = "313") noexcept->void override;
		auto setMvq(const double* vq) noexcept->void override;
		auto setMvm(const double* vm) noexcept->void override;
		auto setMva(const double* va) noexcept->void override;
		auto setMvs(const double* vs) noexcept->void override;
		auto getMve(double* ve, const char *type = "313")const noexcept->void override;
		auto getMvq(double* vq)const noexcept->void override;
		auto getMvm(double* vm)const noexcept->void override;
		auto getMva(double* va)const noexcept->void override;
		auto getMvs(double* vs)const noexcept->void override;
		auto mas()const noexcept->const double6& override;
		auto updMas() noexcept->void override;
		auto setMae(const double* ae, const char *type = "313") noexcept->void override;
		auto setMaq(const double* aq) noexcept->void override;
		auto setMam(const double* am) noexcept->void override;
		auto setMaa(const double* aa) noexcept->void override;
		auto setMas(const double* as) noexcept->void override;
		auto getMae(double* ae, const char *type = "313")const noexcept->void override;
		auto getMaq(double* aq)const noexcept->void override;
		auto getMam(double* am)const noexcept->void override;
		auto getMaa(double* aa)const noexcept->void override;
		auto getMas(double* as)const noexcept->void override;
		auto mfs() const noexcept->const double6& override;
		auto setMfs(const double * mfs) noexcept->void override;

		///***新增
		auto getMpmWrtToolWobj(const double* tool_pm, const double* wobj_pm, double* pm)const noexcept->void override;
		auto setMpmWrtToolWobj(const double* tool_pm, const double* wobj_pm, const double* pm) noexcept->void override;

		virtual ~GeneralMotion();
		explicit GeneralMotion(const std::string &name = "general_motion", Marker *makI = nullptr, Marker *makJ = nullptr, bool active = true);
		CODEIT_REGISTER_TYPE(GeneralMotion);
		CODEIT_DECLARE_BIG_FOUR(GeneralMotion);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};


	class PointMotion final :public CartesianMotion {
	public:
		static auto Dim()->Size { return 3; }
		auto virtual dim() const noexcept ->Size override { return Dim(); }
		auto virtual locCmI() const noexcept->const double* override;
		auto virtual cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const noexcept->void override;
		auto virtual cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void override;
		auto virtual cptCv(double *cv)const noexcept->void override;
		auto virtual cptCa(double *ca)const noexcept->void override;
		auto virtual mpm()const noexcept->const double4x4& override;
		auto virtual updMpm() noexcept->void override;
		auto virtual setMpm(const double *mp) noexcept->void override;
		auto virtual getMpm(double *mp)const noexcept->void override;
		auto virtual mvs()const noexcept->const double6& override;
		auto virtual updMvs() noexcept->void override;
		auto virtual setMvs(const double *mv) noexcept->void override;
		auto virtual getMvs(double *mv)const noexcept->void override;
		auto virtual mas()const noexcept->const double6& override;
		auto virtual updMas() noexcept->void override;
		auto virtual setMas(const double *a) noexcept->void override;
		auto virtual getMas(double *a)const noexcept->void override;
		auto virtual mfs()const noexcept->const double6& override;
		auto virtual setMfs(const double *f) noexcept->void override { setCf(f); }

		virtual ~PointMotion();
		explicit PointMotion(const std::string &name = "point_motion", Marker *makI = nullptr, Marker *makJ = nullptr, bool active = true);
		CODEIT_REGISTER_TYPE(PointMotion);
		CODEIT_DECLARE_BIG_FOUR(PointMotion);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};



	class Force :public Interaction
	{
	public:
		auto virtual cptGlbFs(double *fsI, double *fsJ)const noexcept->void = 0;

		virtual ~Force() = default;
		explicit Force(const std::string &name = "force", Marker *makI = nullptr, Marker *makJ = nullptr, bool active = true) :Interaction(name, makI, makJ, active) {}
		CODEIT_REGISTER_TYPE(Force);
		CODEIT_DEFINE_BIG_FOUR(Force);
	};

	/*
	class Friction :public Element
	{
	public:
		auto frc()const noexcept ->double { return s_vv(frc_coe_.size(), frc_coe_.data(), frc_value_.data()); };
		auto frcValue()const noexcept ->std::tuple<const double *, Size> 
		{ 
			frc_coe_



			
			
			return std::make_tuple(frc_value_.data(), static_cast<Size>(frc_value_.size()));
		}
		auto frcCoe() const noexcept ->std::tuple<const double *, Size> { return std::make_tuple(frc_coe_.data(), static_cast<Size>(frc_coe_.size())); }
		auto setFrcCoe(const double *frc_coe, Size dim) noexcept->void { frc_coe_.assign(frc_coe, frc_coe + dim); frc_value_.resize(dim, 0.0); }
		


		



	private:
		std::vector<double> frc_coe_{0.0, 0.0, 0.0};
		std::vector<double> frc_value_{ 0.0, 0.0, 0.0 };
	};*/

	class RevoluteJoint final :public Joint
	{
	public:
		static auto Dim()->Size { return 5; }
		auto virtual dim() const noexcept->Size override { return Dim(); }
		auto virtual locCmI() const noexcept->const double* override;
		auto virtual cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const noexcept->void override;
		auto virtual cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void override;

		virtual ~RevoluteJoint() = default;
		explicit RevoluteJoint(const std::string &name = "revolute_joint", Marker *makI = nullptr, Marker *makJ = nullptr);
		CODEIT_REGISTER_TYPE(RevoluteJoint);
		CODEIT_DEFINE_BIG_FOUR(RevoluteJoint);
	};
	class PrismaticJoint final :public Joint
	{
	public:
		static auto Dim()->Size { return 5; }
		auto virtual dim() const noexcept->Size override { return Dim(); }
		auto virtual locCmI() const noexcept->const double* override;
		auto virtual cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const noexcept->void override;
		auto virtual cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void override;
		
		virtual ~PrismaticJoint() = default;
		explicit PrismaticJoint(const std::string &name = "prismatic_joint", Marker *makI = nullptr, Marker *makJ = nullptr);
		CODEIT_REGISTER_TYPE(PrismaticJoint);
		CODEIT_DEFINE_BIG_FOUR(PrismaticJoint);
	};
	class UniversalJoint final :public Joint
	{
	public:
		static auto Dim()->Size { return 4; }
		auto virtual dim() const noexcept->Size override { return Dim(); }
		auto virtual locCmI() const noexcept->const double* override;
		auto virtual cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const noexcept->void override;
		auto virtual cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void override;
		auto virtual cptGlbCmFromPm(double *cmI, double *cmJ, const double *makI_pm, const double *makJ_pm)const noexcept->void override;
		auto virtual cptCa(double *ca)const noexcept->void override;

		virtual ~UniversalJoint();
		explicit UniversalJoint(const std::string &name = "universal_joint", Marker *makI = nullptr, Marker *makJ = nullptr);
		CODEIT_REGISTER_TYPE(UniversalJoint);
		CODEIT_DECLARE_BIG_FOUR(UniversalJoint);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	class SphericalJoint final :public Joint
	{
	public:
		static auto Dim()->Size { return 3; }
		auto virtual dim() const noexcept->Size override { return Dim(); }
		auto virtual locCmI() const noexcept->const double* override;
		auto virtual cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const noexcept->void override;
		auto virtual cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void override;

		virtual ~SphericalJoint() = default;
		explicit SphericalJoint(const std::string &name = "spherical_joint", Marker *makI = nullptr, Marker *makJ = nullptr);
		CODEIT_REGISTER_TYPE(SphericalJoint);
		CODEIT_DEFINE_BIG_FOUR(SphericalJoint);
	};

	class GeneralForce final :public Force
	{
	public:
		auto virtual cptGlbFs(double *fsI, double *fsJ)const noexcept->void override { s_vc(6, fce_value_, fsI); s_vi(6, fce_value_, fsJ); }
		auto setFce(const double *value) noexcept->void { std::copy(value, value + 6, fce_value_); }
		auto fce()const noexcept->const double* { return fce_value_; }

		virtual ~GeneralForce() = default;
		explicit GeneralForce(const std::string &name = "general_force", Marker *makI = nullptr, Marker *makJ = nullptr) : Force(name, makI, makJ) {};
		CODEIT_REGISTER_TYPE(GeneralForce);
		CODEIT_DEFINE_BIG_FOUR(GeneralForce);

	private:
		double fce_value_[6]{ 0 };
	};
	class SingleComponentForce final :public Force
	{
	public:
		auto virtual saveXml(core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement &xml_ele)->void override;
		auto virtual cptGlbFs(double *fsI, double *fsJ)const noexcept->void override;
		auto setComponentID(Size id) noexcept->void { component_axis_ = id; }
		auto setFce(double value) noexcept->void { std::fill_n(fce_value_, 6, 0); fce_value_[component_axis_] = value; }
		auto setFce(double value, Size componentID) noexcept->void { this->component_axis_ = componentID; setFce(value); }
		auto fce()const noexcept->double { return fce_value_[component_axis_]; }

		virtual ~SingleComponentForce() = default;
		explicit SingleComponentForce(const std::string &name = "single_component_force", Marker *makI = nullptr, Marker *makJ = nullptr, Size componentID = 0);
		CODEIT_REGISTER_TYPE(SingleComponentForce);
		CODEIT_DEFINE_BIG_FOUR(SingleComponentForce);

	private:
		Size component_axis_;
		double fce_value_[6]{ 0 };
	};

	class ExternMotion :public DynEle
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		auto mp() const noexcept->double;
		auto setMp(double mp) noexcept->void;
		auto mv() const noexcept->double;
		auto setMv(double mv) noexcept->void;
		auto ma() const noexcept->double;
		auto setMa(double ma) noexcept->void;
		auto mf() const noexcept->double;
		auto setMf(double mf) noexcept->void;
		auto mfDyn() const noexcept->double;
		auto setMfDyn(double mf_dyn) noexcept->void;
		auto mfFrc() const noexcept->double;
		auto frcCoe() const noexcept ->const double3&;
		auto setFrcCoe(const double* frc_coe) noexcept->void;
		auto frcZeroCheck()const noexcept ->double { return 1e-3; }
		auto mpOffset()const noexcept->double;
		auto setMpOffset(double mp_offset)noexcept->void;
		auto mpFactor()const noexcept->double;
		auto setMpFactor(double mp_factor)noexcept->void;
		
		virtual ~ExternMotion();
		explicit ExternMotion(const std::string& name = "extern_motion", const double* frc_coe = nullptr, double mp_offset = 0.0, double mp_factor = 1.0, bool active = true);
		CODEIT_REGISTER_TYPE(ExternMotion);
		CODEIT_DECLARE_BIG_FOUR(ExternMotion);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
	
	
	
	/// @}
}

#endif
