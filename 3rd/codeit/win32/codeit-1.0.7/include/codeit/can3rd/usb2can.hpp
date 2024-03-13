#ifndef __USB2CAN_h__
#define __USB2CAN_h__
#include <string>
#include<codeit/core/object.hpp>
#include"codeit/can3rd/controlcan.h"

namespace codeit::controller
{
	#define UNS8   unsigned char
	#define UNS16  unsigned short
	#define UNS32  unsigned long

	class CanMessage {
	public:
		UNS16 cob_id;	/**< message's ID */
		UNS8 rtr;		/**< remote transmission request. (0 if not rtr message, 1 if rtr message) */
		UNS8 len;		/**< message's length (0 to 8) */
		UNS8 data[8]; /**< message's datas */
		char messageStr[1024] = { 0 };
		auto message2String()->char*;
	};
	
	typedef void* CAN_HANDLE;
	typedef void* CAN_PORT;
	
	class CanDriver {
	public:
		struct CanBoard {
			Size device_id;
			Size device_type;
			std::string baudrate0; /**< The board baudrate */
			Size node_num0;
			std::string baudrate1; /**< The board baudrate */
			Size node_num1;
		};
		static const CanBoard defaultCanBoard;
	
		auto canBoard()const->const CanBoard;
		auto setCanBoard(CanBoard& board)->void;
		auto isVirtual()->bool;
		auto setVirtual(bool is_virtual = true)->void;
		UNS8 canReceive_driver(CAN_HANDLE inst, VCI_CAN_OBJ* pCanObj, int& buf_lenth);
		UNS8 canSend_driver(CAN_HANDLE inst, CanMessage&m, bool check_return=true);
		int canOpen_driver();
		int canClose_driver(CAN_HANDLE inst);
		UNS8 canChangeBaudRate_driver(CAN_HANDLE inst, CanMessage&m);

		virtual ~CanDriver();
		explicit CanDriver(const std::string& name = "can_driver", const CanBoard& board=defaultCanBoard);
		//CODEIT_REGISTER_TYPE(CanDriver);
		CanDriver(const CanDriver& other) = delete;
		CanDriver(CanDriver&& other) = delete;
		CanDriver& operator=(const CanDriver& other) = delete;
		CanDriver& operator=(CanDriver&& other) = delete;
	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;

	};

}

#endif