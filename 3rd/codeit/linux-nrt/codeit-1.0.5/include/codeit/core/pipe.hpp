#ifndef CODEIT_CORE_PIPE_H
#define CODEIT_CORE_PIPE_H

#include <codeit/core/object.hpp>
#include <codeit/core/msg.hpp>

namespace codeit::core
{
	class Pipe :public codeit::core::Object
	{
	public:
		auto virtual loadXml(const core::XmlElement &xml_ele)->void override;
		auto sendMsg(const core::MsgBase &)->bool;
		auto recvMsg(core::MsgBase &)->bool;

		virtual ~Pipe();
		Pipe(const std::string &name = "pipe", std::size_t pool_size = 16384);
		Pipe(const Pipe&) = delete;
		Pipe(Pipe&&);
		Pipe& operator=(const Pipe&) = delete;
		Pipe& operator=(Pipe&&);
		CODEIT_REGISTER_TYPE(Pipe);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};
}

#endif
