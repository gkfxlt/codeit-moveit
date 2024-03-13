#ifndef CODEIT_CORE_SERIALPORT_H_
#define CODEIT_CORE_SERIALPORT_H_
#include <codeit/core/basictype.hpp>
#include <codeit/core/object.hpp>
#include <codeit/core/msg.hpp> 

#define byte win_byte_override
#ifdef WIN32
#include <windows.h>
#undef min
#undef max
#undef byte
#endif
#ifdef UNIX
 struct termios;
#endif
namespace codeit::core
{
	class SerialPort : public Object
	{
	public:
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		enum State
        {
            IDLE = 0,
            WORKING,
        };
		auto startServer()->void;
        auto stop()->void;
        bool WriteData(const char* pData, Size length);
        bool ReadChar(char& cRecved);
        auto recvRawData(char* data, Size size)->void;
        auto state()->State;
        

        enum BaudRate{
                BR0 = 0000000,
                BR50 = 0000001,
                BR75 = 0000002,
                BR110 = 0000003,
                BR134 = 0000004,
                BR150 = 0000005,
                BR200 = 0000006,
                BR300 = 0000007,
                BR600 = 0000010,
                BR1200 = 0000011,
                BR1800 = 0000012,
                BR2400 = 0000013,
                BR4800 = 0000014,
                BR9600 = 0000015,
                BR19200 = 0000016,
                BR38400 = 0000017,
                BR57600 = 0010001,
                BR115200 = 0010002,
                BR230400 = 0010003,
                BR460800 = 0010004,
                BR500000 = 0010005,
                BR576000 = 0010006,
                BR921600 = 0010007,
                BR1000000 = 0010010,
                BR1152000 = 0010011,
                BR1500000 = 0010012,
                BR2000000 = 0010013,
                BR2500000 = 0010014,
                BR3000000 = 0010015,
                BR3500000 = 0010016,
                BR4000000 = 0010017
            };
        enum DataBits {
                DataBits5,
                DataBits6,
                DataBits7,
                DataBits8,
            };
        enum StopBits {
                StopBits1,
                StopBits2
            };
        enum Parity {
                ParityNone,
                ParityEven,
                PariteMark,
                ParityOdd,
                ParitySpace
            };
        
  
		auto setOnReceivedData(std::function<int(SerialPort*, const char* data, int size)> = nullptr)->void;
		auto setOnOpenConnection(std::function<int(SerialPort*)> = nullptr)->void;
		auto setOnCloseConnection(std::function<int(SerialPort*)> = nullptr)->void;
        
#ifdef UNIX
        struct ComOptions {
                std::string path;
                BaudRate baudRate;
                DataBits dataBits;
                StopBits stopBits;
                Parity parity;
                bool xon;
                bool xoff;
                bool xany;
                int vmin;
                int vtime;
            };
        static const ComOptions defaultComOptions;
        bool open();
        bool open(const std::string& path, const ComOptions& options);
        void termiosOptions(termios& tios, const ComOptions& options);
        bool isOpen() const;
        auto portNo()const->const std::string&;
        auto setPortNo(const std::string&)->void;
        auto setBaud(const BaudRate&)->void;
        auto baud()const->const BaudRate&;
        static BaudRate BaudRateMake(unsigned long baudrate);
        static DataBits DataBitsMake(unsigned long databits);
        static StopBits StopBitsMake(unsigned long stopbits);
        static Parity ParityMake(unsigned long parity);
        void ClosePort();
        static std::vector<std::string > list();
#endif    

#ifdef WIN32
        struct ComOptions {
            UINT portNo_;
            UINT baud_;
            char  parity_;
            UINT  databits_;
            UINT  stopsbits_;
            DWORD dwCommEvents_;
        };
        static const ComOptions defaultComOptions;
        auto portNo()const->const UINT&;
        auto setPortNo(const UINT&)->void;
        auto baud()const->const UINT&;
        auto setBaud(const UINT&)->void;
        auto parity()const->const char&;
        auto setParity(const char&)->void;
        auto databits()const->const UINT&;
        auto setDatabits(const UINT&)->void;
        auto stopsbits()const->const UINT&;
        auto setStopsbits(const UINT&)->void;
        auto dwCommEvents()const->const DWORD&;
        auto setDwCommEvents(const DWORD&)->void;
        bool OpenPort(UINT  portNo);
        void ClosePort();
        UINT GetBytesInCOM();
        bool InitPort(UINT  portNo, UINT  baud, char  parity, UINT  databits, UINT  stopsbits, DWORD dwCommEvents);
        bool InitPort(UINT  portNo, const LPDCB& plDCB);
		#endif		
        
        
        virtual ~SerialPort();
        SerialPort(const std::string& name="COM", const ComOptions& options= defaultComOptions, bool open_recv_thread = true);
        SerialPort(const SerialPort& other) = delete;
		SerialPort(SerialPort&& other) = default;
		SerialPort& operator=(const SerialPort& other) = delete;
		SerialPort& operator=(SerialPort&& other) = default;
        CODEIT_REGISTER_TYPE(SerialPort);

	private:
		struct Imp;
		const std::unique_ptr<Imp> imp_;

	};

#ifdef UNIX
    bool operator==(const SerialPort::ComOptions& lhs, const SerialPort::ComOptions& rhs);
    bool operator!=(const SerialPort::ComOptions& lhs, const SerialPort::ComOptions& rhs);
#endif
}


#endif
