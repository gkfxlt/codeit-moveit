#ifndef CODEIT_CONTROLLER_MASTER_H_
#define CODEIT_CONTROLLER_MASTER_H_

#include <any>
#include <thread>
#include<codeit/core/basictype.hpp>
#include<codeit/core/core.hpp>

namespace codeit::controller
{
	class Slave : public core::Object
	{
	public:
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto virtual send()->void {}
		auto virtual recv()->void {}
		auto phyId()const->std::uint16_t;
		auto setPhyId(std::uint16_t phy_id)->void;
		auto slaId()const->std::uint16_t { return static_cast<std::uint16_t>(id()); }

		virtual ~Slave();
		explicit Slave(const std::string& name = "slave", std::uint16_t phy_id = 0);
		CODEIT_REGISTER_TYPE(Slave);
		CODEIT_DECLARE_BIG_FOUR(Slave);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
	};

	class Master :public core::Object {
	public:
		struct RtStasticsData
		{
			double avg_time_consumed;
			std::int64_t max_time_consumed;
			std::int64_t max_time_occur_count;
			std::int64_t min_time_consumed;
			std::int64_t min_time_occur_count;
			std::int64_t total_count;
			std::int64_t overrun_count;
		};
		enum { MAX_MSG_SIZE = 8192 };
		static auto Type()->const std::string& { static const std::string type("Master"); return std::ref(type); }
		auto virtual type() const->const std::string & override { return Type(); }
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;

		// used only in non-rt thread //
		auto virtual init()->void;
		auto virtual start()->void;
		auto virtual stop()->void;
		auto setControlStrategy(std::function<void()> strategy)->void;
		auto setSamplePeriodNs(int period_ns)->void;
		auto samplePeriodNs()const ->int;

		// used in rt thread //
		auto logFile(const char* file_name)->void;
		auto logFileRawName(const char* file_name)->void;
		auto lout()->core::MsgStream&;
		auto mout()->core::MsgStream&;
		auto slaveAtAbs(Size id)->Slave& { return slavePool().at(id); }
		auto slaveAtAbs(Size id)const->const Slave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtAbs(id); }
		auto slaveAtPhy(Size id)->Slave&;
		auto slaveAtPhy(Size id)const->const Slave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtPhy(id); }
		auto slavePool()->core::ObjectPool<Slave>&;
		auto slavePool()const->const core::ObjectPool<Slave>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slavePool(); }
		auto rtHandle()->std::any&;
		auto rtHandle()const->const std::any& { return const_cast<std::decay_t<decltype(*this)>*>(this)->rtHandle(); }
		auto resetRtStasticData(RtStasticsData* stastics, bool is_new_data_include_this_count)->void;

		virtual ~Master();
		explicit Master(const std::string& name = "master");
		Master(const Master& other) = delete;
		Master(Master&& other) = delete;
		Master& operator=(const Master& other) = delete;
		Master& operator=(Master&& other) = delete;

	protected:
		auto virtual send()->void { for (auto& s : slavePool())s.send(); }
		auto virtual recv()->void { for (auto& s : slavePool())s.recv(); }
		auto virtual release()->void {}

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
		
		friend class Slave;
		friend class PdoEntry;
	};


	///***ÐÂÔöNrtMaster**//
	class NrtMaster :public core::Object {
	public:
		struct NrtStasticsData
		{
			double avg_time_consumed;
			std::int64_t max_time_consumed;
			std::int64_t max_time_occur_count;
			std::int64_t min_time_consumed;
			std::int64_t min_time_occur_count;
			std::int64_t total_count;
			std::int64_t overrun_count;
		};
		enum { MAX_MSG_SIZE = 8192 };
		static auto Type()->const std::string& { static const std::string type("NrtMaster"); return std::ref(type); }
		auto virtual type() const->const std::string & override { return Type(); }
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;

		// used only in non-rt thread //
		auto virtual init()->void;
		auto virtual start()->void;
		auto virtual stop()->void;
		auto setControlStrategy(std::function<void(Size)> strategy)->void;
		auto setSamplePeriodNs(int period_ns)->void;
		auto samplePeriodNs()const ->int;
		auto setNrtId(Size nrt_id_);
		auto nrtId()const ->Size;
		auto setPackSize(Size pack_size);
		auto packSize()const ->Size;
		// used in rt thread //
		/*auto logFile(const char* file_name)->void;
		auto logFileRawName(const char* file_name)->void;
		auto lout()->core::MsgStream&;
		auto mout()->core::MsgStream&;*/
		auto slaveAtAbs(Size id)->Slave& { return slavePool().at(id); }
		auto slaveAtAbs(Size id)const->const Slave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtAbs(id); }
		auto slaveAtPhy(Size id)->Slave&;
		auto slaveAtPhy(Size id)const->const Slave& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slaveAtPhy(id); }
		auto slavePool()->core::ObjectPool<Slave>&;
		auto slavePool()const->const core::ObjectPool<Slave>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->slavePool(); }
		auto rtHandle()->std::any&;
		auto rtHandle()const->const std::any& { return const_cast<std::decay_t<decltype(*this)>*>(this)->rtHandle(); }
		auto resetNrtStasticData(NrtStasticsData* stastics, bool is_new_data_include_this_count)->void;

		virtual ~NrtMaster();
		explicit NrtMaster(const std::string& name = "nrt_master", const Size& packSize=0, const Size& nrt_id=0);
		NrtMaster(const NrtMaster& other) = delete;
		NrtMaster(NrtMaster&& other) = delete;
		NrtMaster& operator=(const NrtMaster& other) = delete;
		NrtMaster& operator=(NrtMaster&& other) = delete;

	protected:
		auto virtual send()->void { for (auto& s : slavePool())s.send(); }
		auto virtual recv()->void { for (auto& s : slavePool())s.recv(); }
		auto virtual release()->void {}

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;

		friend class Slave;

	};
}


#endif