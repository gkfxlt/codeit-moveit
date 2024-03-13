#ifndef CODEIT_DYNAMIC_MODEL_SINGULARITY_H_
#define CODEIT_DYNAMIC_MODEL_SINGULARITY_H_

#include <codeit/model/model_basic.hpp>

namespace codeit::model
{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	class SigularArbiter :public Element
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto virtual isSingular(codeit::model::Model& model_)->std::int32_t;
		auto setTol(const double& tol_)->void;
		auto tol()->double;
		
		virtual ~SigularArbiter();
		explicit SigularArbiter(const std::string& name = "sigular_arbiter", const double& tolerance=1.0e-3);
		CODEIT_REGISTER_TYPE(SigularArbiter);
		CODEIT_DECLARE_BIG_FOUR(SigularArbiter);
	private:
		double tolerance_{ 0 };

	};
	class PumaSigularArbiter : public SigularArbiter
	{
	public:
		auto virtual isSingular(codeit::model::Model& model_)->std::int32_t;

		virtual ~PumaSigularArbiter();
		explicit PumaSigularArbiter(const std::string& name = "puma_sigular_arbiter", const double& tolerance = 1.0e-3);
		CODEIT_REGISTER_TYPE(PumaSigularArbiter);
		CODEIT_DECLARE_BIG_FOUR(PumaSigularArbiter);

	};

	class URSigularArbiter : public SigularArbiter
	{
	public:
		auto virtual isSingular(codeit::model::Model& model_)->std::int32_t;

		virtual ~URSigularArbiter();
		explicit URSigularArbiter(const std::string& name = "ur_sigular_arbiter", const double& tolerance = 1.0e-3);
		CODEIT_REGISTER_TYPE(URSigularArbiter);
		CODEIT_DECLARE_BIG_FOUR(URSigularArbiter);

	};

	class ScaraSigularArbiter : public SigularArbiter
	{
	public:
		auto virtual isSingular(codeit::model::Model& model_)->std::int32_t;

		virtual ~ScaraSigularArbiter();
		explicit ScaraSigularArbiter(const std::string& name = "scara_sigular_arbiter", const double& tolerance = 1.0e-3);
		CODEIT_REGISTER_TYPE(ScaraSigularArbiter);
		CODEIT_DECLARE_BIG_FOUR(ScaraSigularArbiter);

	};

	class IiwaSigularArbiter : public SigularArbiter
	{
	public:
		auto virtual isSingular(codeit::model::Model& model_)->std::int32_t;

		virtual ~IiwaSigularArbiter();
		explicit IiwaSigularArbiter(const std::string& name = "iiwa_sigular_arbiter", const double& tolerance = 1.0e-3);
		CODEIT_REGISTER_TYPE(IiwaSigularArbiter);
		CODEIT_DECLARE_BIG_FOUR(IiwaSigularArbiter);

	};

	class UrThreeSigularArbiter : public SigularArbiter
	{
	public:
		auto virtual isSingular(codeit::model::Model& model_)->std::int32_t;

		virtual ~UrThreeSigularArbiter();
		explicit UrThreeSigularArbiter(const std::string& name = "ur_three_sigular_arbiter", const double& tolerance = 1.0e-3);
		CODEIT_REGISTER_TYPE(UrThreeSigularArbiter);
		CODEIT_DECLARE_BIG_FOUR(UrThreeSigularArbiter);

	};
	///
	/// @}
}



#endif
