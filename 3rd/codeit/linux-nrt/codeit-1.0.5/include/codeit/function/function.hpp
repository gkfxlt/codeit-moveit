#ifndef CODEIT_FUNCTION_H_
#define CODEIT_FUNCTION_H_
#include<codeit/function/basefunc.hpp>
#include<codeit/function/motionfunc.hpp>
#include<codeit/function/jogfunc.hpp>
#include<codeit/function/systemfunc.hpp>
#include<codeit/function/clbfunc.hpp>
#include<codeit/function/egmfunc.hpp>
#include<codeit/function/iofunc.hpp>
#include<codeit/function/demofunc.hpp>
#include<codeit/function/screwfunc.hpp>
#include<codeit/function/mathfunc.hpp>
#include<codeit/function/triggfunc.hpp>
#include<codeit/function/sensorfunc.hpp>
#include<codeit/function/dvrkfunc.hpp>
#include<codeit/function/canfunc.hpp>
#include<codeit/function/cstfunc.hpp>
#include<codeit/function/quadfunc.hpp>
#include<codeit/function/canfunc.hpp>
#include<codeit/function/nrtfunc.hpp>
#include<codeit/function/ftplanfunc.hpp>

namespace codeit::function {
	auto defaultFuncRoot(codeit::function::FuncRoot& cmdRoot)->void;
}

#endif