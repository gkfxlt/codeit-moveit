#ifndef CODEIT_CORE_Socket_H_
#define CODEIT_CORE_Socket_H_
#include <codeit/core/basictype.hpp>
#include <codeit/core/object.hpp>
#include <codeit/core/msg.hpp>

namespace codeit::core
{
	class Socket : public Object
	{
	public:
		
		std::mutex result_mutex;
		std::list<core::Msg> result_list;
		enum State
		{
			IDLE = 0,
			WAITING_FOR_CONNECTION,
			WORKING,
		};
		enum TYPE
		{
			TCP,
			UDP,
			WEB,
			UDP_RAW,
			WEB_RAW
		};

	public:
		auto virtual loadXml(const core::XmlElement &xml_ele)->void override;
		auto virtual saveXml(core::XmlElement &xml_ele) const->void override;
		auto isConnected()->bool;
		auto state()->State;
		auto startServer(const std::string &port = std::string())->void;
		auto startClient(const std::string& port = std::string())->void;
		auto connect(const std::string &remote_ip = std::string(), const std::string &port = std::string())->void;
		auto stop()->void;
		auto sendMsg(const core::MsgBase &data)->void;
		auto sendRawData(const char *data, int size)->void;
		auto port()const->const std::string &;
		auto setPort(const std::string &port)->void;
		auto remoteIP()const->const std::string &;
		auto setRemoteIP(const std::string &remote_ip)->void;
		auto isServer()const->const bool&;
		auto setIsServer(const bool& is_server)->void;
		auto connectType()const->TYPE;
		auto setConnectType(const TYPE type)->void;
		auto setOnReceivedMsg(std::function<int(Socket*, core::Msg &)> = nullptr)->void;
		auto setOnReceivedRawData(std::function<int(Socket*, const char *data, int size)> = nullptr)->void;
		auto setOnReceivedConnection(std::function<int(Socket*, const char* remote_ip, int remote_port)> = nullptr)->void;
		auto setOnLoseConnection(std::function<int(Socket*)> = nullptr)->void;
		auto setOnGetConnection(std::function<int(Socket*)> = nullptr)->void;
		////****新增**//
		auto setOnSendRawData(std::function<int(Socket*)> = nullptr)->void;
		auto recvRawData(char* data, Size size)->void;
		auto openRecvThread()const->const bool&;
		auto setOpenRecvThread(const bool& flag)->void;
		auto openSendThread()const->const bool&;
		auto setOpenSendThread(const bool& flag)->void;

		virtual ~Socket();
		Socket(const std::string &name = "Socket", const std::string& remote_ip = "", const std::string& port = "", TYPE type = TCP, bool open_recv_thread=true, bool is_server=true, bool open_send_thread = false);
		Socket(const Socket & other) = delete;
		Socket(Socket && other) = default;
		Socket &operator=(const Socket& other) = delete;
		Socket &operator=(Socket&& other) = default;
		CODEIT_REGISTER_TYPE(Socket);

	private:
		struct Imp;
		const std::unique_ptr<Imp> imp_;
	};
}


#endif
