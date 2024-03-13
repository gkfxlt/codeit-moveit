#ifndef CODEIT_SYSTEM_SERVER_H_
#define CODEIT_SYSTEM_SERVER_H_

#include<codeit/system/interface.hpp>
#include<codeit/controller/controller.hpp>
#include<codeit/core/core.hpp>
#include<codeit/core/basictype.hpp>
#include<codeit/function/basefunc.hpp>
#include<codeit/model/allmodel.hpp>
#include<codeit/sensor/sensor.hpp>
#include<codeit/system/errorinfo.hpp>

#include <atomic>
#include<future>
#include<map>

using namespace codeit::controller;
using namespace codeit::function;
using namespace codeit::model;
using namespace codeit::core;

namespace codeit::system {
	class ControlSystem : public core::Object {

	public:

		using PreCallback = std::add_pointer<void(ControlSystem&)>::type;
		using PostCallback = std::add_pointer<void(ControlSystem&)>::type;

		// members //
		static ControlSystem& instance() { static ControlSystem Instance; return Instance; }
		auto virtual loadXml(const core::XmlElement& xml_ele)->void override;
		auto virtual saveXml(core::XmlElement& xml_ele) const->void override;

		//template<typename T = aris::model::Model, typename... Args>
		//auto makeModel(Args&&... args)->void { this->resetModel(new T(std::forward<Args>(args)...)); }
		template<typename T = core::ObjectPool<codeit::model::Model>, typename... Args>
		auto makeModelPool(Args&&... args)->void { this->resetModelPool(new T(std::forward<Args>(args)...)); }
		//template<typename T = core::ObjectPool<codeit::model::IOModel>, typename... Args>
		//auto makeIOModelPool(Args&&... args)->void { this->resetIOModelPool(new T(std::forward<Args>(args)...)); }
		template<typename T = codeit::function::FuncRoot, typename... Args>
		auto makeFuncRoot(Args&&... args)->void { this->resetFuncRoot(new T(std::forward<Args>(args)...)); }
		template<typename T = core::ObjectPool<codeit::sensor::Sensor>, typename... Args>
		auto makeSensorPool(Args&&... args)->void { this->resetSensorPool(new T(std::forward<Args>(args)...)); }
		template<typename T = codeit::controller::Controller, typename... Args>
		auto makeController(Args&&... args)->void { this->resetController(new T(std::forward<Args>(args)...)); }
		template<typename T = core::ObjectPool<codeit::controller::SocketController>, typename... Args>
		auto makeNrtControllerPool(Args&&... args)->void { this->resetNrtControllerPool(new T(std::forward<Args>(args)...)); }
		template<typename T = core::ObjectPool<codeit::system::ErrorInfo>, typename... Args>
		auto makeErrorInfoPool(Args&&... args)->void { this->resetErrorInfoPool(new T(std::forward<Args>(args)...)); }


		auto model()->model::Model&;
		auto model()const->const model::Model& { return const_cast<ControlSystem*>(this)->model(); }
		auto funcRoot()->codeit::function::FuncRoot&;
		auto funcRoot()const->const function::FuncRoot& { return const_cast<ControlSystem*>(this)->funcRoot(); }
		auto sensorPool()->core::ObjectPool<sensor::Sensor>&;
		auto sensorPool()const->const core::ObjectPool<sensor::Sensor>& { return const_cast<ControlSystem*>(this)->sensorPool(); }
		auto controller()->codeit::controller::Controller&;
		auto controller()const->const controller::Controller& { return const_cast<ControlSystem*>(this)->controller(); }
		auto nrtControllerPool()->core::ObjectPool<controller::NrtController>&;
		auto nrtControllerPool()const->const core::ObjectPool<controller::NrtController>& { return const_cast<ControlSystem*>(this)->nrtControllerPool(); }
		auto modelPool()->core::ObjectPool<codeit::model::Model>&;
		auto modelPool()const->const core::ObjectPool<codeit::model::Model>& { return const_cast<ControlSystem*>(this)->modelPool(); }
		/*auto ioModelPool()->core::ObjectPool<codeit::model::IOModel>&;
		auto ioModelPool()const->const core::ObjectPool<codeit::model::IOModel>& { return const_cast<ControlSystem*>(this)->ioModelPool(); }*/


		auto interfacePool()->core::ObjectPool<codeit::system::Interface>&;
		auto interfacePool()const->const core::ObjectPool<codeit::system::Interface>& { return const_cast<ControlSystem*>(this)->interfacePool(); }
		auto interfaceRoot()->InterfaceRoot&;
		auto interfaceRoot()const->const InterfaceRoot& { return const_cast<ControlSystem*>(this)->interfaceRoot(); }
		auto errorinfoPool()->core::ObjectPool<codeit::system::ErrorInfo>&;
		auto errorinfoPool()const->const core::ObjectPool<codeit::system::ErrorInfo>& { return const_cast<ControlSystem*>(this)->errorinfoPool(); }
		auto errorinfoVer()->int&;
		auto errorinfoVer()const->const int& { return const_cast<ControlSystem*>(this)->errorinfoVer(); }
		auto errorMap()->std::map<std::int32_t, std::string>&;
		auto errorMap()const->const std::map<std::int32_t, std::string>& { return const_cast<ControlSystem*>(this)->errorMap(); }
		auto setErrorinfoVer(int)->void;


		auto resetSensorPool(core::ObjectPool<codeit::sensor::Sensor>* sensor_pool)->void;
		auto resetModelPool(core::ObjectPool<codeit::model::Model>* model_pool)->void;
		//auto resetIOModelPool(core::ObjectPool<codeit::model::IOModel>* iomodel_pool)->void;
		auto resetFuncRoot(codeit::function::FuncRoot* cmd_root)->void;
		auto resetController(codeit::controller::Controller* controller)->void;
		auto resetNrtControllerPool(core::ObjectPool<codeit::controller::NrtController>* controller_pool)->void;
		auto resetErrorInfoPool(core::ObjectPool<codeit::system::ErrorInfo>* errorinfo_pool)->void;



		// operation in RT & NRT context //
		auto setRtPlanPreCallback(PreCallback pre_callback)->void;
		auto setRtPlanPostCallback(PostCallback post_callback)->void;
		auto setNrtPlanPreCallback(PreCallback pre_callback)->void;
		auto setNrtPlanPostCallback(PostCallback post_callback)->void;
		auto running()->bool;
		auto globalCount()->std::int64_t;

		auto setProRateNow(Size, double)->void;
		auto setProRateNew(Size, double)->void;
		auto proRateNow(Size)->double;
		auto proRateNew(Size)->double;
		auto varyRateImmediate(Size)->int;//改变速度标志,立即生效
		auto setVaryRateImmediate(Size, int)->void;
		auto varyRatePersist(Size)->int;//速度由now变成new的过程变量
		auto setVaryRatePersist(Size, int)->void;
		auto varyRateCount(Size)->std::uint64_t;//速度由now变成new需要的count数
		auto setVaryRateCount(Size, std::uint64_t)->void;
		auto ut(Size)->double&;
		auto setUt(Size, double)->void;
		auto dut(Size)->double&;
		auto setDut(Size, double)->void;
		auto ddut(Size)->double&;
		auto setDdut(Size, double)->void;
		auto varyCount(Size)->std::int64_t;//改变速度时的指令count值
		auto setVaryCount(Size, std::int64_t)->void;


		auto setBreakRtNow(Size, double)->void;
		auto setBreakRtNew(Size, double)->void;
		auto breakRtNow(Size)->double;
		auto breakRtNew(Size)->double;
		auto varyBreakRtImmediate(Size)->int;//改变速度标志,立即生效
		auto setVaryBreakRtImmediate(Size, int)->void;
		auto varyBreakRtPersist(Size)->int;//速度由now变成new的过程变量
		auto setVaryBreakRtPersist(Size, int)->void;
		auto varyBreakRtCount(Size)->std::uint64_t;//速度由now变成new需要的count数
		auto setVaryBreakRtCount(Size, std::uint64_t)->void;
		auto utBreakRt(Size)->double&;
		auto dutBreakRt(Size)->double&;
		auto ddutBreakRt(Size)->double&;
		auto varyCountBreakRt(Size)->std::int64_t;//改变速度时的指令count值
		auto setVaryCountBreakRt(Size, std::int64_t)->void;

		auto breakByWait(Size)->bool;
		auto setBreakByWait(Size, bool)->void;

		auto currentExecutePlanRt()->codeit::function::BasisFunc*;
		auto motionState()->std::uint64_t;//模型的状态
		auto setMotionState(std::uint64_t)->void;//模型的状态
		auto controlMode()->std::uint8_t;//手臂控制模式
		auto setControlMode(std::uint8_t)->void;//设置手臂控制模式
		auto globalSwitchFlag()->std::int32_t;//全局切换标志
		auto setGlobalSwitchFlag(std::int32_t)->void;//设置全局切换标志
		auto masterLostConnection()->bool;//从站失去连接
		auto setMasterLostConnection(bool)->void;//设置从站失去连接

		// operation in NRT context //
		auto open()->void;
		auto close()->void;
		auto runCmdLine()->void;
		auto executeCmd(std::vector<std::pair<std::string_view, std::function<void(codeit::function::BasisFunc&, Size)>>>)->std::vector<std::shared_ptr<codeit::function::BasisFunc>>;
		auto executeCmd(std::string_view cmd_str, std::function<void(codeit::function::BasisFunc&, Size)> post_callback = nullptr)->std::shared_ptr<codeit::function::BasisFunc>;
		auto executeCmdInCmdLine(std::vector<std::pair<std::string_view, std::function<void(codeit::function::BasisFunc&, Size)>>>)->std::vector<std::shared_ptr<codeit::function::BasisFunc>>;
		auto executeCmdInCmdLine(std::string_view cmd_string, std::function<void(codeit::function::BasisFunc&, Size)> post_callback = nullptr)->std::shared_ptr<codeit::function::BasisFunc>;

		auto init()->void;
		auto start()->void;
		auto stop()->void;
		auto nrtStart()->void;
		auto nrtStop()->void;
		auto rngCheck()->bool;
		auto setRngCheck(bool)->void;
		auto dynamicEngine()->bool;
		auto setDynamicEngine(bool)->void;
		auto collisionDetection()->bool;
		auto setCollisionDetection(bool)->void;
		auto checkMasterConnection()->bool;
		auto setCheckMasterConnection(bool)->void;
		auto setOnIOTrigger(std::function<void(ControlSystem&)> func)->void;


		auto currentExecutePlan()->std::shared_ptr<codeit::function::BasisFunc>;
		auto currentCollectPlan()->std::shared_ptr<codeit::function::BasisFunc>;
		auto waitForAllExecution()->void;
		auto waitForAllCollection()->void;
		auto getRtData(const std::function<void(ControlSystem&, const codeit::function::BasisFunc* target, std::any&)>& get_func, std::any& data)->void;
		auto errorCode()const->int;
		auto errorMsg()const->const char*;
		auto clearError()->void;

		auto setMaxRtLog(Size)->void;
		auto setMaxLog(Size)->void;
		auto maxRtLog()->Size;
		auto maxLog()->Size;
		void scanFile(std::vector<std::string> &fileList);
		void deleteFile(std::vector<std::string> &fileList);

		void triggerCmd();
		void checkMasterConnect();
		CODEIT_REGISTER_TYPE(ControlSystem);

	private:

		//////////////////////////////////////Control System Info Start////////////////////////////////////
		string manufacture;
		string version;
		string ip;
		//////////////////////////////////////Control System Info Stop////////////////////////////////////


		~ControlSystem();
		ControlSystem();
		ControlSystem(const ControlSystem&) = delete;
		ControlSystem& operator=(const ControlSystem&) = delete;

	private:

		struct Imp;
		std::unique_ptr<Imp> imp_;
	};
}


#endif
