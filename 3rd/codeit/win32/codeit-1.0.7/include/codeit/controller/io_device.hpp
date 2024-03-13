#ifndef CODEIT_CONTROLLER_IO_DEVICE_H_
#define CODEIT_CONTROLLER_IO_DEVICE_H_
#include <codeit/controller/master_slave.hpp>
namespace codeit::controller
{
	class IO : public virtual Slave
	{
	public:
		enum IO_TYPE : std::uint64_t
		{
			DI_DEVICE = 0x01ULL << 0,
			DO_DEVICE = 0x01ULL << 1,
			AI_DEVICE = 0x01ULL << 2,
			AO_DEVICE = 0x01ULL << 3,
		};
		auto ioId()const->Size;

		auto virtual ioType()->IO_TYPE { return io_type; }
		auto virtual setIoType(IO_TYPE type_)->void { io_type = type_; }

		virtual ~IO() = default;
		explicit IO(const std::string& name = "io", std::uint16_t phy_id = 0);
		IO(const IO& other);
		IO(IO&& other) = delete;
		IO& operator=(const IO& other);
		IO& operator=(IO&& other) = delete;

	private:
		/*struct Imp;
		core::ImpPtr<Imp> imp_;*/
		Size io_id_;
		IO_TYPE io_type;
		friend class Controller;
		friend class NrtController;
	};

	class DI : public virtual IO
	{
	public:
		auto virtual actualDi()const->bool =0;
		auto virtual setDi(bool)->void = 0;
		virtual ~DI() = default;
		explicit DI(const std::string& name = "di", std::uint16_t phy_id = 0);
		DI(const DI& other);
		DI(DI&& other) = delete;
		DI& operator=(const DI& other);
		DI& operator=(DI&& other) = delete;

	private:
		/*struct Imp;
		core::ImpPtr<Imp> imp_;*/
		
		friend class Controller;
		friend class NrtController;
	};

	class DO : public virtual IO
	{
	public:

		auto virtual actualDo()const->bool = 0;
		auto virtual setDo(bool)->void = 0;
		virtual ~DO() = default;
		explicit DO(const std::string& name = "do", std::uint16_t phy_id = 0);
		DO(const DO& other);
		DO(DO&& other) = delete;
		DO& operator=(const DO& other);
		DO& operator=(DO&& other) = delete;

	private:
		/*struct Imp;
		core::ImpPtr<Imp> imp_;*/
		
		friend class Controller;
		friend class NrtController;
	};

	class AI : public IO
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		auto virtual actualAi()const->double = 0;
		auto virtual setAi(double)->void = 0;
		auto virtual resolution()const->double;
		auto virtual setResolution(double res_)->void;
		virtual ~AI() = default;
		explicit AI(const std::string& name = "ai", std::uint16_t phy_id = 0, double res_=1);
		AI(const AI& other);
		AI(AI&& other) = delete;
		AI& operator=(const AI& other);
		AI& operator=(AI&& other) = delete;

	private:
		/*struct Imp;
		core::ImpPtr<Imp> imp_;*/
		double res;
		friend class Controller;
		friend class NrtController;
	};

	class AO : public IO
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;

		auto virtual actualAo()const->double = 0;
		auto virtual setAo(double)->void = 0;
		auto virtual resolution()const->double;
		auto virtual setResolution(double res_)->void;
		virtual ~AO() = default;
		explicit AO(const std::string& name = "ao", std::uint16_t phy_id = 0, double res_=1);
		AO(const AO& other);
		AO(AO&& other) = delete;
		AO& operator=(const AO& other);
		AO& operator=(AO&& other) = delete;

	private:
		/*struct Imp;
		core::ImpPtr<Imp> imp_;*/
		double res;
		friend class Controller;
		friend class NrtController;
	};

	class SubSystem: public virtual Slave
	{
	public:
		
		auto subSystemId()const->Size;
		auto virtual actualCmd()->std::string = 0;
		auto virtual setCmd(std::string)->void = 0;
		
		virtual ~SubSystem() = default;
		explicit SubSystem(const std::string& name = "subsystem", std::uint16_t phy_id = 0);
		SubSystem(const SubSystem& other);
		SubSystem(SubSystem&& other) = delete;
		SubSystem& operator=(const SubSystem& other);
		SubSystem& operator=(SubSystem&& other) = delete;

	private:
		Size subsys_id_;
	
		friend class Controller;
		friend class NrtController;
	};
}

#endif