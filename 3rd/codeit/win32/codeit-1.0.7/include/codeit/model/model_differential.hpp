#ifndef CODEIT_DIFFERENTIAL_MODEL_H_
#define CODEIT_DIFFERENTIAL_MODEL_H_

#include <codeit/model/model_solver.hpp>
#include <codeit/function/basefunc.hpp>
//模型加在非实时线程里
namespace codeit::model {

	struct TwoWheelModelParam
	{
		// DH PARAM //
		double W1{ 0 };	//	L1
		double W2{ 0 };	//	L2
		//double L3{ 0 };
		double H1{ 0 };	//	L1
		double H2{ 0 };	//	L2
		double R1{ 0 };
		double R2{ 0 };

		// TOOL 0, by default is 321 type
		double tool0_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::string tool0_pe_type;

		// BASE wrt REF, by default is 321 type 
		double base2ref_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::string base2ref_pe_type;

		// inertia vector, size must be 6
		std::vector<std::array<double, 10> > iv_vec;

		// mot friction vector, size must be 6
		std::vector<std::array<double, 6> > mot_frc_vec;

		// 驱动在零位处的偏移，以及系数
		double mp_offset[6] = {0};	// mp_real = (mp_theoretical - mp_offset) * mp_factor
		double mp_factor[6] = {0};


	};

	auto createModelTwoWheelRobot(const TwoWheelModelParam& param, std::string name = "model")->std::unique_ptr<codeit::model::Model>;

	auto TwoWheelRobotModelForward(TwoWheelModelParam& param, const double dt, const double* vel_wheel, const double* pose_body_last_pe, double* pose_world_frame, double* vel_chassis_frame, double *vel_world_frame)->bool;

	auto TwoWheelRobotModelInverse(TwoWheelModelParam& param, const double* vel_world_frame, double* vel_chassis_frame, const double* pose_body_last_pe, double* vel_wheel)->bool;

	class TwoWheelRobotForwardKinematicSolver :public codeit::model::ForwardKinematicSolver
	{
	public:
		auto virtual saveXml(core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement &xml_ele)->void override;
		auto virtual allocateMemory()->void override;
		auto virtual kinVel()->void override;
		auto setWhichRoot(int root_of_0_to_7)->void;

		virtual ~TwoWheelRobotForwardKinematicSolver() = default;
		explicit TwoWheelRobotForwardKinematicSolver(const std::string& name = "TwoWheel_robot_forward_solver");
		CODEIT_REGISTER_TYPE(TwoWheelRobotForwardKinematicSolver);
		CODEIT_DECLARE_BIG_FOUR(TwoWheelRobotForwardKinematicSolver);

	public:
		struct Imp;
		core::ImpPtr<Imp> imp_;
		codeit::controller::NrtController * nrtcontroller_ptr;
	};

	class TwoWheelRobotInverseKinematicSolver :public codeit::model::InverseKinematicSolver
	{
	public:
		auto virtual saveXml(core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const core::XmlElement &xml_ele)->void override;
		auto virtual allocateMemory()->void override;
		auto virtual kinVel()->void override;
		auto setWhichRoot(int root_of_0_to_7)->void;

		

		virtual ~TwoWheelRobotInverseKinematicSolver() = default;
		explicit TwoWheelRobotInverseKinematicSolver(const std::string& name = "TwoWheel_robot_inverse_solver");
		CODEIT_REGISTER_TYPE(TwoWheelRobotInverseKinematicSolver);
		CODEIT_DECLARE_BIG_FOUR(TwoWheelRobotInverseKinematicSolver);

	private:
		struct Imp;
		core::ImpPtr<Imp> imp_;
		codeit::controller::NrtController * nrtcontroller_ptr;
	};

}

#endif // !UR_MODEL_H_

