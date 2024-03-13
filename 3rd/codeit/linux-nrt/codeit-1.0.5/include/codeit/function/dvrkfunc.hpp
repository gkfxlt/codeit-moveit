#ifndef CODEIT_DVRK_FUNCTION_H_
#define CODEIT_DVRK_FUNCTION_H_
#include<codeit/function/basefunc.hpp>

namespace codeit::system { class ControlSystem; }
namespace codeit::function {

	class VerifyDvrk : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~VerifyDvrk();
		explicit VerifyDvrk(const std::string& name = "VerifyDvrk");
		CODEIT_REGISTER_TYPE(VerifyDvrk);
		CODEIT_DECLARE_BIG_FOUR(VerifyDvrk);

	};

	class DragMaster : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~DragMaster();
		explicit DragMaster(const std::string& name = "DragMaster");
		CODEIT_REGISTER_TYPE(DragMaster);
		CODEIT_DECLARE_BIG_FOUR(DragMaster);
	};



	class MasterSlaveSync : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~MasterSlaveSync();
		explicit MasterSlaveSync(const std::string& name = "MasterSlaveSync");
		CODEIT_REGISTER_TYPE(MasterSlaveSync);
		CODEIT_DECLARE_BIG_FOUR(MasterSlaveSync);

	};
}
#endif
