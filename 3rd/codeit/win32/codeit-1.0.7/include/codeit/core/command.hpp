#ifndef CODEIT_CORE_COMMAND_H_
#define CODEIT_CORE_COMMAND_H_

#include <map>

#include <codeit/core/object.hpp>

namespace codeit::core
{

	class Command;

	class ParamBase :public ObjectPool<ParamBase>
	{
	public:
		auto command()const->const Command&;

		virtual ~ParamBase();
		explicit ParamBase(const std::string& name = "param_base");
		CODEIT_REGISTER_TYPE(ParamBase);
		CODEIT_DECLARE_BIG_FOUR(ParamBase);

	protected:
		struct Imp;
		ImpPtr<Imp> imp_;
		friend class Command;
	};

	/// \class aris::core::Param
	///  命令解析的参数节点
	/// 
	/// 
	/// 
	class Param final :public ParamBase
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto abbreviation()const->char;
		auto setAbbreviation(char abbreviation)->void;
		auto defaultValue()const->const std::string&;
		auto setDefaultValue(const std::string& default_value)->void;

		virtual ~Param();
		explicit Param(const std::string& name = "param", const std::string& default_param = "", char abbrev = 0);
		CODEIT_REGISTER_TYPE(Param);
		CODEIT_DECLARE_BIG_FOUR(Param);

	protected:
		struct Imp;
		ImpPtr<Imp> imp_;

		friend class Command;
		friend class CommandParser;
	};
	class UniqueParam final :public ParamBase
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto defaultParam()const->const std::string&;

		virtual ~UniqueParam();
		explicit UniqueParam(const std::string& name = "unique_param", const std::string& default_param = "");
		CODEIT_REGISTER_TYPE(UniqueParam);
		CODEIT_DECLARE_BIG_FOUR(UniqueParam);

	protected:
		struct Imp;
		ImpPtr<Imp> imp_;

		friend class Command;
	};
	class GroupParam final : public ParamBase
	{
	public:
		virtual ~GroupParam();
		explicit GroupParam(const std::string& name = "group_param");
		CODEIT_REGISTER_TYPE(GroupParam);
		CODEIT_DECLARE_BIG_FOUR(GroupParam);
	};
	class Command :public ObjectPool<ParamBase>
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto defaultParam()const->const std::string&;
		auto findParam(const std::string& param_name)const->const Param* { return const_cast<std::decay_t<decltype(*this)>*>(this)->findParam(param_name); }
		auto findParam(const std::string& param_name)->Param*;

		virtual ~Command();
		explicit Command(const std::string& name = "command", const std::string& default_param = "");
		CODEIT_REGISTER_TYPE(Command);
		CODEIT_DECLARE_BIG_FOUR(Command);

	private:
		struct Imp;
		ImpPtr<Imp> imp_;

		friend class CommandParser;
		friend class ParamBase;
	};
	class CommandParser :public Object
	{
	public:
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto commandPool()->ObjectPool<Command>&;
		auto commandPool()const->const ObjectPool<Command>&;
		auto init()->void;
		auto parse(std::string_view command_string)->std::tuple<std::string_view, std::map<std::string_view, std::string_view>>;

		virtual ~CommandParser();
		explicit CommandParser(const std::string& name = "command_parser");
		CODEIT_REGISTER_TYPE(CommandParser);
		CODEIT_DECLARE_BIG_FOUR(CommandParser);

	private:
		struct Imp;
		ImpPtr<Imp> imp_;
	};

	/// @}
}


#endif
