#ifndef CODEIT_DYNAMIC_MODEL_H_
#define CODEIT_DYNAMIC_MODEL_H_

#include <codeit/core/expression_calculator.hpp>

#include <codeit/model/model_solver.hpp>
#include <codeit/model/model_simulation.hpp>
#include <codeit/model/plan.hpp>
#include<codeit/model/model_planner.hpp>/////////*******新增
#include<codeit/model/model_singularity.hpp>/////****新增
#include<codeit/model/model_calibrator.hpp>//////****新增
#include<codeit/model/model_infrgn.hpp>//////****新增
#include<codeit/model/iomodel.hpp>

namespace codeit::model
{

	class Model :public core::Object
	{
	public:

		auto virtual inverseKinematics()noexcept->int;
		auto virtual forwardKinematics()noexcept->int;
		/*auto virtual inverseKinematicsVel()noexcept->int;
		auto virtual forwardKinematicsVel()noexcept->int;
		auto virtual inverseDynamics()noexcept->int;
		auto virtual forwardDynamics()noexcept->int;*/

		auto virtual inputPosSize()const noexcept->Size;
		auto virtual inputDim()const noexcept->Size;
		auto virtual getInputPos(double *mp)const noexcept->void;
		auto virtual setInputPos(const double *mp)noexcept->void;
		auto virtual getInputVel(double *mv)const noexcept->void;
		auto virtual setInputVel(const double *mv)noexcept->void;
		auto virtual getInputAcc(double *ma)const noexcept->void;
		auto virtual setInputAcc(const double *ma)noexcept->void;
		auto virtual getInputFce(double *mf)const noexcept->void;
		auto virtual setInputFce(const double *mf)noexcept->void;

		auto virtual outputPosSize()const noexcept->Size;
		auto virtual outputDim()const noexcept->Size;
		auto virtual getOutputPos(double *mp)const noexcept->void;
		auto virtual setOutputPos(const double *mp)noexcept->void;
		auto virtual getOutputVel(double *mv)const noexcept->void;
		auto virtual setOutputVel(const double *mv)noexcept->void;
		auto virtual getOutputAcc(double *ma)const noexcept->void;
		auto virtual setOutputAcc(const double *ma)noexcept->void;
		auto virtual getOutputFce(double *mf)const noexcept->void;
		auto virtual setOutputFce(const double *mf)noexcept->void;


		auto virtual loadXml(const core::XmlElement &xml_ele)->void override;
		auto virtual saveXml(core::XmlElement &xml_ele)const->void override;
		auto virtual init()->void;

		auto time()const->double;
		auto setTime(double time)->void;
		auto dynaEngine()const->bool;
		auto setDynaEngine(bool flag)->void;
		/// @{
		auto calculator()->core::Calculator&;
		auto calculator()const ->const core::Calculator& { return const_cast<std::decay_t<decltype(*this)> *>(this)->calculator(); }
		auto environment()->codeit::model::Environment&;
		auto environment()const ->const codeit::model::Environment& { return const_cast<std::decay_t<decltype(*this)> *>(this)->environment(); }
		auto variablePool()->core::ObjectPool<Variable, Element>&;
		auto variablePool()const->const core::ObjectPool<Variable, Element>& { return const_cast<std::decay_t<decltype(*this)> *>(this)->variablePool(); }
		auto partPool()->core::ObjectPool<Part, Element>&;
		auto partPool()const->const core::ObjectPool<Part, Element>& { return const_cast<std::decay_t<decltype(*this)> *>(this)->partPool(); }
		auto jointPool()->core::ObjectPool<Joint, Element>&;
		auto jointPool()const->const core::ObjectPool<Joint, Element>& { return const_cast<std::decay_t<decltype(*this)> *>(this)->jointPool(); }
		auto motionPool()->core::ObjectPool<Motion, Element>&;
		auto motionPool()const->const core::ObjectPool<Motion, Element>& { return const_cast<std::decay_t<decltype(*this)> *>(this)->motionPool(); }
		auto generalMotionPool()->core::ObjectPool<CartesianMotion, Element>&;
		auto generalMotionPool()const->const core::ObjectPool<CartesianMotion, Element>& { return const_cast<std::decay_t<decltype(*this)> *>(this)->generalMotionPool(); }
		auto forcePool()->core::ObjectPool<Force, Element>&;
		auto forcePool()const->const core::ObjectPool<Force, Element>& { return const_cast<std::decay_t<decltype(*this)> *>(this)->forcePool(); }
		auto solverPool()->core::ObjectPool<Solver, Element>&;
		auto solverPool()const->const core::ObjectPool<Solver, Element>& { return const_cast<std::decay_t<decltype(*this)> *>(this)->solverPool(); }
		auto simulatorPool()->core::ObjectPool<Simulator, Element>&;
		auto simulatorPool()const->const core::ObjectPool<Simulator, Element>& { return const_cast<std::decay_t<decltype(*this)> *>(this)->simulatorPool(); }
		auto simResultPool()->core::ObjectPool<SimResult, Element>&;
		auto simResultPool()const->const core::ObjectPool<SimResult, Element>& { return const_cast<std::decay_t<decltype(*this)> *>(this)->simResultPool(); }
		auto calibratorPool()->core::ObjectPool<Calibrator, Element>&;
		auto calibratorPool()const->const core::ObjectPool<Calibrator, Element>& { return const_cast<std::decay_t<decltype(*this)> *>(this)->calibratorPool(); }
		auto jointPlannerPool()->core::ObjectPool<JointPlanner, Element>&;
		auto jointPlannerPool()const->const core::ObjectPool<JointPlanner, Element>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->jointPlannerPool(); }
		auto moveLPlannerPool()->core::ObjectPool<MoveLPlanner, Element>&;
		auto moveLPlannerPool()const->const core::ObjectPool<MoveLPlanner, Element>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->moveLPlannerPool(); }
		auto moveCPlannerPool()->core::ObjectPool<MoveCPlanner, Element>&;
		auto moveCPlannerPool()const->const core::ObjectPool<MoveCPlanner, Element>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->moveCPlannerPool(); }
		auto moveSPlannerPool()->core::ObjectPool<MoveSPlanner, Element>&;
		auto moveSPlannerPool()const->const core::ObjectPool<MoveSPlanner, Element>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->moveSPlannerPool(); }
		auto moveLLPlannerPool()->core::ObjectPool<MoveLLPlanner, Element>&;
		auto moveLLPlannerPool()const->const core::ObjectPool<MoveLLPlanner, Element>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->moveLLPlannerPool(); }
		auto servoJPlannerPool()->core::ObjectPool<ServoJPlanner, Element>&;
		auto servoJPlannerPool()const->const core::ObjectPool<ServoJPlanner, Element>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->servoJPlannerPool(); }
		auto egmPlannerPool()->core::ObjectPool<EGMPlanner, Element>&;
		auto egmPlannerPool()const->const core::ObjectPool<EGMPlanner, Element>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->egmPlannerPool(); }

		auto pointPool()->core::ObjectPool<PointData, Element>&;
		auto pointPool()const->const core::ObjectPool<PointData, Element>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->pointPool(); }
		auto toolPool()->core::ObjectPool<ToolData, Element>&;
		auto toolPool()const->const core::ObjectPool<ToolData, Element>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->toolPool(); }
		auto wobjPool()->core::ObjectPool<WobjData, Element>&;
		auto wobjPool()const->const core::ObjectPool<WobjData, Element>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->wobjPool(); }

		auto sigularArbiterPool()->core::ObjectPool<SigularArbiter, Element>&;
		auto sigularArbiterPool()const->const core::ObjectPool<SigularArbiter, Element>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->sigularArbiterPool(); }
		auto interferRNGPool()->core::ObjectPool<InterferRNG, Element>&;
		auto interferRNGPool()const->const core::ObjectPool<InterferRNG, Element>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->interferRNGPool(); }
		auto externMotionPool()->core::ObjectPool<ExternMotion, Element>&;
		auto externMotionPool()const->const core::ObjectPool<ExternMotion, Element>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->externMotionPool(); }

		
		
		auto idleJointPlanner()->JointPlanner*;
		auto idleJointPlanner()const->const JointPlanner* { return const_cast<std::decay_t<decltype(*this)>*>(this)->idleJointPlanner(); }
		auto idleMoveLPlanner()->MoveLPlanner*;
		auto idleMoveLPlanner()const->const MoveLPlanner* { return const_cast<std::decay_t<decltype(*this)>*>(this)->idleMoveLPlanner(); }
		auto idleMoveCPlanner()->MoveCPlanner*;
		auto idleMoveCPlanner()const->const MoveCPlanner* { return const_cast<std::decay_t<decltype(*this)>*>(this)->idleMoveCPlanner(); }
		auto idleMoveSPlanner()->MoveSPlanner*;
		auto idleMoveSPlanner()const->const MoveSPlanner* { return const_cast<std::decay_t<decltype(*this)>*>(this)->idleMoveSPlanner(); }
		auto idleMoveLLPlanner()->MoveLLPlanner*;
		auto idleMoveLLPlanner()const->const MoveLLPlanner* { return const_cast<std::decay_t<decltype(*this)>*>(this)->idleMoveLLPlanner(); }
		auto idleServoJPlanner()->ServoJPlanner*;
		auto idleServoJPlanner()const->const ServoJPlanner* { return const_cast<std::decay_t<decltype(*this)>*>(this)->idleServoJPlanner(); }
		auto idleEGMPlanner()->EGMPlanner*;
		auto idleEGMPlanner()const->const EGMPlanner* { return const_cast<std::decay_t<decltype(*this)>*>(this)->idleEGMPlanner(); }


		auto ground()->Part&;
		auto ground()const->const Part& { return const_cast<std::decay_t<decltype(*this)> *>(this)->ground(); }
		/// @}
		/// @{
		auto addPartByPm(const double*pm, const double *prt_iv = nullptr)->Part&;
		auto addPartByPe(const double*pe, const char* eul_type, const double *prt_iv = nullptr)->Part&;
		auto addPartByPq(const double*pq, const double *prt_iv = nullptr)->Part&;
		auto addRevoluteJoint(Part &first_part, Part &second_part, const double *position, const double *axis)->RevoluteJoint&;
		auto addPrismaticJoint(Part &first_part, Part &second_part, const double *position, const double *axis)->PrismaticJoint&;
		auto addUniversalJoint(Part &first_part, Part &second_part, const double *position, const double *first_axis, const double *second_axis)->UniversalJoint&;
		auto addSphericalJoint(Part &first_part, Part &second_part, const double *position)->SphericalJoint&;
		auto addMotion(Joint &joint)->Motion&;
		auto addMotion()->Motion&;
		auto addGeneralMotionByPm(Part &end_effector, Coordinate &reference, const double* pm)->GeneralMotion&;
		auto addGeneralMotionByPe(Part &end_effector, Coordinate &reference, const double* pe, const char* eul_type)->GeneralMotion&;
		auto addGeneralMotionByPq(Part &end_effector, Coordinate &reference, const double* pq)->GeneralMotion&;
		auto addPointMotion(Part &end_effector, Part &reference, const double* pos_in_ground)->PointMotion&;

		///////*************在原有基础上新增***************/////////
		auto ioPool()->core::ObjectPool<IOElement, Object>&;
		auto ioPool()const->const core::ObjectPool<IOElement, Object>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->ioPool(); }
		auto subsysPool()->core::ObjectPool<SubSysElement, Object>&;
		auto subsysPool()const->const core::ObjectPool<SubSysElement, Object>& { return const_cast<std::decay_t<decltype(*this)>*>(this)->subsysPool(); }
		auto isRtModel()const->bool;
		auto setIsRtModel(bool flag)->void;
		
		
		/// @}
		virtual ~Model();
		explicit Model(const std::string &name = "model");
		CODEIT_REGISTER_TYPE(Model);
		CODEIT_DECLARE_BIG_FOUR(Model);


	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
		friend class Motion;
	};
	/// @}

	auto defaultModelData(codeit::model::Model& model)->void;

}

#endif
