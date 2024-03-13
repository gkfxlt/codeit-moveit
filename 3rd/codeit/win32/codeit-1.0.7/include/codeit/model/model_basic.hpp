#ifndef CODEIT_DYNAMIC_MODEL_BASIC_H_
#define CODEIT_DYNAMIC_MODEL_BASIC_H_

#include <codeit/core/object.hpp>
#include <codeit/core/expression_calculator.hpp>
#include <codeit/core/data_type.hpp>
#include <codeit/model/screw.hpp>

namespace codeit::model
{
	using double6x6 = double[6][6];
	using double4x4 = double[4][4];
	using double3 = double[3];
	using double6 = double[6];
	using double7 = double[7];
	using double10 = double[10];
	using doubleMAX_DOFS = double[MAX_DOFS];
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///

	class Model;

	class Element :public core::Object
	{
	public:
		auto model()noexcept->Model& { return *ancestor<Model>(); }
		auto model()const noexcept->const Model& { return const_cast<std::decay_t<decltype(*this)> *>(this)->model(); }
		auto attributeMatrix(const core::XmlElement &xml_ele, const std::string &attribute_name)const->core::Matrix;
		auto attributeMatrix(const core::XmlElement &xml_ele, const std::string &attribute_name, const core::Matrix& default_value)const->core::Matrix;
		auto attributeMatrix(const core::XmlElement &xml_ele, const std::string &attribute_name, Size m, Size n)const->core::Matrix;
		auto attributeMatrix(const core::XmlElement &xml_ele, const std::string &attribute_name, Size m, Size n, const core::Matrix& default_value)const->core::Matrix;

		~Element() = default;
		explicit Element(const std::string &name = "element") :Object(name) {}
		CODEIT_REGISTER_TYPE(Element);
		CODEIT_DEFINE_BIG_FOUR(Element);
	};
	class DynEle : public Element
	{
	public:
		auto virtual saveXml(core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement &xml_ele)->void override;
		auto active() const noexcept->bool { return active_; }
		auto activate(bool active = true)noexcept->void { active_ = active; }

		virtual ~DynEle() = default;
		explicit DynEle(const std::string &name, bool active = true) : Element(name), active_(active) {};
		CODEIT_REGISTER_TYPE(DynEle);
		CODEIT_DEFINE_BIG_FOUR(DynEle);

	private:
		bool active_;
	};

	class Environment final :public Element
	{
	public:
		auto virtual saveXml(core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement &xml_ele)->void override;
		auto gravity()const noexcept->const double6& { return gravity_; }
		auto setGravity(const double *gravity)noexcept->void { s_vc(6, gravity, gravity_); }

		virtual ~Environment() = default;
		explicit Environment(const std::string &name = "dyn_ele") :Element(name) {}
		CODEIT_REGISTER_TYPE(Environment);
		CODEIT_DEFINE_BIG_FOUR(Environment);

	private:
		double gravity_[6]{ 0, -9.8, 0, 0, 0, 0 };
	};

	class Variable :public Element
	{
	public:
		auto virtual saveXml(core::XmlElement &xml_ele) const->void override;
		auto virtual toString() const->std::string { return ""; }

		virtual ~Variable() = default;
		explicit Variable(const std::string &name = "variable") : Element(name) {}
		CODEIT_REGISTER_TYPE(Variable);
		CODEIT_DEFINE_BIG_FOUR(Variable);
	};
	template<typename VariableType> class VariableTemplate : public Variable
	{
	public:
		auto data()->VariableType& { return data_; }
		auto data()const->const VariableType& { return data_; }

		virtual ~VariableTemplate() = default;
		explicit VariableTemplate(const std::string &name = "variable_template", const VariableType &data = VariableType(), bool active = true) : Variable(name), data_(data) {}
		VariableTemplate(const VariableTemplate &other) = default;
		VariableTemplate(VariableTemplate &&other) = default;
		VariableTemplate& operator=(const VariableTemplate &other) = default;
		VariableTemplate& operator=(VariableTemplate &&other) = default;

	private:
		VariableType data_;
	};
	class MatrixVariable final : public VariableTemplate<core::Matrix>
	{
	public:
		auto virtual loadXml(const core::XmlElement &xml_ele)->void override;
		auto virtual toString() const->std::string override { return data().toString(); }

		virtual ~MatrixVariable() = default;
		explicit MatrixVariable(const std::string &name = "matrix_variable", const core::Matrix &data = core::Matrix()) : VariableTemplate(name, data) {}
		CODEIT_REGISTER_TYPE(MatrixVariable);
		CODEIT_DEFINE_BIG_FOUR(MatrixVariable);
	};
	class StringVariable final : public VariableTemplate<std::string>
	{
	public:
		auto virtual loadXml(const core::XmlElement &xml_ele)->void override;
		auto virtual toString() const->std::string override { return data(); }

		virtual ~StringVariable() = default;
		explicit StringVariable(const std::string &name = "string_variable", const std::string &data = "") : VariableTemplate(name, data) {}
		CODEIT_REGISTER_TYPE(StringVariable);
		CODEIT_DEFINE_BIG_FOUR(StringVariable);
	};


	////////////////********新增示教点变量*******///////
	class PointData :public Element
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		auto tool()const noexcept->const core::Tool& { return tool_; }
		auto wobj()const noexcept->const core::Wobj& { return wobj_; }
		auto robotTarget()const noexcept->const core::RobotTarget& { return robotTarget_; }
		auto jointTarget()const noexcept->const core::JointTarget& { return jointTarget_; }

		virtual ~PointData() = default;
		explicit PointData(const std::string& name ="point_data", \
			const core::JointTarget* jointTarget = nullptr, const core::RobotTarget* robotTarget = nullptr, \
			const core::Tool* tool = nullptr, const core::Wobj* wobj = nullptr);
		CODEIT_REGISTER_TYPE(PointData);
		CODEIT_DEFINE_BIG_FOUR(PointData);
	private:
		core::Tool tool_;
		core::Wobj wobj_;
		core::JointTarget jointTarget_;
		core::RobotTarget robotTarget_;
	};
	
	class ToolData :public Element
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto tool()const noexcept->const core::Tool& { return tool_; }
		
		virtual ~ToolData() = default;
		explicit ToolData(const std::string& name = "tool_data", \
			const core::Tool* tool = nullptr);
		CODEIT_REGISTER_TYPE(ToolData);
		CODEIT_DEFINE_BIG_FOUR(ToolData);
	private:
		core::Tool tool_;
	};

	class WobjData :public Element
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto wobj() const noexcept->const core::Wobj& { return wobj_; }

		virtual ~WobjData() = default;
		explicit WobjData(const std::string& name = "wobj_data", \
			const core::Wobj* wobj = nullptr);
		CODEIT_REGISTER_TYPE(WobjData);
		CODEIT_DEFINE_BIG_FOUR(WobjData);
	private:
		core::Wobj wobj_;
	};
}

#endif
