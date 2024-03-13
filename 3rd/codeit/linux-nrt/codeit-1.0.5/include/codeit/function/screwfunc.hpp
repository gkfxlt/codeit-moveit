#ifndef CODEIT_SCREW_FUNCTION_H_
#define CODEIT_SCREW_FUNCTION_H_
#include<codeit/function/basefunc.hpp>
namespace codeit::function {
	class Offs : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		virtual ~Offs();
		explicit Offs(const std::string& name = "Offs");
		CODEIT_REGISTER_TYPE(Offs);
		CODEIT_DECLARE_BIG_FOUR(Offs);
	};

	class Pe2Pq : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		virtual ~Pe2Pq();
		explicit Pe2Pq(const std::string& name = "Pe2Pq");
		CODEIT_REGISTER_TYPE(Pe2Pq);
		CODEIT_DECLARE_BIG_FOUR(Pe2Pq);
	};

	class Pa2Pq : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		virtual ~Pa2Pq();
		explicit Pa2Pq(const std::string& name = "Pa2Pq");
		CODEIT_REGISTER_TYPE(Pa2Pq);
		CODEIT_DECLARE_BIG_FOUR(Pa2Pq);
	};


}



#endif
