#ifndef CODEIT_DYNAMIC_MODEL_INF_RGN_H_
#define CODEIT_DYNAMIC_MODEL_INF_RGN_H_

#include <codeit/model/model_basic.hpp>

namespace codeit::model
{
	class Model;
	class InterferRNG :public Element
	{
	public:
		enum RNG_TYPE
		{
			INSIDE,
			OUTSIDE
		};
		enum RNG_PRI
		{
			HIGH,
			LOW
		};
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto virtual region()const->const double* =0;
		auto virtual setRegion(const double*)->void = 0;
		
		auto diName()const->const std::string&;
		auto setDiName(const string& di_name_)->void;
		auto doName()const->const std::string&;
		auto setDoName(const string& do_name_)->void;
		auto masterName()const->const std::string&;
		auto setMasterName(const string& master_name_)->void;
		auto rngType()const->const RNG_TYPE&;
		auto setRngType(const RNG_TYPE& rng_type_)->void;
		auto rngPri()const->const RNG_PRI&;
		auto setRngPri(const RNG_PRI& rng_pri_)->void;

		auto virtual isInRng(const Model& model)->bool = 0;
		virtual ~InterferRNG();
		explicit InterferRNG(const std::string& name = "InterferRNG",const std::string&\
		master_name="com", const std::string& do_name="com_do0", const std::string& di_name = "com_doi",\
		const RNG_TYPE& rng_type = INSIDE, const RNG_PRI& rng_pri = HIGH);
		CODEIT_REGISTER_TYPE(InterferRNG);
		CODEIT_DECLARE_BIG_FOUR(InterferRNG);
	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;

	};
	class CartInterferRNG : public InterferRNG
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		auto virtual region()const->const double*;
		auto virtual setRegion(const double* region=nullptr)->void;

		auto toolPm()const->const double*;
		auto setToolPm(const double* pm = nullptr)->void;
		auto wobjPm()const->const double*;
		auto setWobjPm(const double* pm = nullptr)->void;


		auto virtual isInRng(const Model&)->bool;
		virtual ~CartInterferRNG();
		explicit CartInterferRNG(const std::string& name = "InterferRNG", const std::string& \
			master_name = "com", const std::string& do_name = "com_do0", const std::string& di_name = "com_doi", \
			const RNG_TYPE& rng_type = INSIDE, const RNG_PRI& rng_pri = HIGH, const double* region=nullptr, const double* tool_pm=nullptr, const double* wobj_pm=nullptr);
		CODEIT_REGISTER_TYPE(CartInterferRNG);
		CODEIT_DECLARE_BIG_FOUR(CartInterferRNG);
	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;

	};



	class AxisInterferRNG : public InterferRNG
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		auto virtual region()const->const double*;
		auto virtual setRegion(const double* region=nullptr)->void;
		auto virtual isInRng(const Model&)->bool;
		virtual ~AxisInterferRNG();
		explicit AxisInterferRNG(const std::string& name = "AxisInterferRNG", const std::string& \
			master_name = "com", const std::string& do_name = "com_do0", const std::string& di_name = "com_doi", \
			const RNG_TYPE& rng_type = INSIDE, const RNG_PRI& rng_pri = HIGH, const double* region = nullptr);
		CODEIT_REGISTER_TYPE(AxisInterferRNG);
		CODEIT_DECLARE_BIG_FOUR(AxisInterferRNG);
	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};


}



#endif
