#ifndef CODEIT_JOG_FUNCTION_H_
#define CODEIT_JOG_FUNCTION_H_
#include<codeit/function/motionfunc.hpp>

namespace codeit::system { class ControlSystem; }
namespace codeit::function {

	class JogJ1 : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~JogJ1();
		explicit JogJ1(const std::string &name = "JogJ1");
		CODEIT_REGISTER_TYPE(JogJ1);
		CODEIT_DECLARE_BIG_FOUR(JogJ1);
	};

	class JogJ2 : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~JogJ2();
		explicit JogJ2(const std::string &name = "JogJ2");
		CODEIT_REGISTER_TYPE(JogJ2);
		CODEIT_DECLARE_BIG_FOUR(JogJ2);
	};

	class JogJ3 : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~JogJ3();
		explicit JogJ3(const std::string &name = "JogJ3");
		CODEIT_REGISTER_TYPE(JogJ3);
		CODEIT_DECLARE_BIG_FOUR(JogJ3);
	};


	class JogJ4 : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~JogJ4();
		explicit JogJ4(const std::string &name = "JogJ4");
		CODEIT_REGISTER_TYPE(JogJ4);
		CODEIT_DECLARE_BIG_FOUR(JogJ4);
	};


	class JogJ5 : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~JogJ5();
		explicit JogJ5(const std::string &name = "JogJ5");
		CODEIT_REGISTER_TYPE(JogJ5);
		CODEIT_DECLARE_BIG_FOUR(JogJ5);
	};


	class JogJ6 : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~JogJ6();
		explicit JogJ6(const std::string &name = "JogJ6");
		CODEIT_REGISTER_TYPE(JogJ6);
		CODEIT_DECLARE_BIG_FOUR(JogJ6);
	};

	class JogTX : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~JogTX();
		explicit JogTX(const std::string &name = "JogTX");
		CODEIT_REGISTER_TYPE(JogTX);
		CODEIT_DECLARE_BIG_FOUR(JogTX);
	};


	class JogTY : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~JogTY();
		explicit JogTY(const std::string &name = "JogTY");
		CODEIT_REGISTER_TYPE(JogTY);
		CODEIT_DECLARE_BIG_FOUR(JogTY);
	};


	class JogTZ : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~JogTZ();
		explicit JogTZ(const std::string &name = "JogTZ");
		CODEIT_REGISTER_TYPE(JogTZ);
		CODEIT_DECLARE_BIG_FOUR(JogTZ);
	};


	class JogRX : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~JogRX();
		explicit JogRX(const std::string &name = "JogRX");
		CODEIT_REGISTER_TYPE(JogRX);
		CODEIT_DECLARE_BIG_FOUR(JogRX);
	};


	class JogRY : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~JogRY();
		explicit JogRY(const std::string &name = "JogRY");
		CODEIT_REGISTER_TYPE(JogRY);
		CODEIT_DECLARE_BIG_FOUR(JogRY);
	};


	class JogRZ : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~JogRZ();
		explicit JogRZ(const std::string &name = "JogRZ");
		CODEIT_REGISTER_TYPE(JogRZ);
		CODEIT_DECLARE_BIG_FOUR(JogRZ);
	};



	class JogL : public BasisFunc
	{
	public:
		auto virtual prepareNrt(BasisFunc&, int)->void;
		auto virtual executeRT(BasisFunc&, int)->int;
		auto virtual collectNrt(BasisFunc&, int)->void;

		virtual ~JogL();
		explicit JogL(const std::string &name = "JogL");
		CODEIT_REGISTER_TYPE(JogL);
		CODEIT_DECLARE_BIG_FOUR(JogL);
	};

}

#endif
