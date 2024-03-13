#ifndef CODEIT_BASICTYPE_H_
#define CODEIT_BASICTYPE_H_
#include <cstddef>
#include <any>
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <list>
#include <atomic>
#include <algorithm>
using namespace std;

#define MAX_DOFS  (10)//一个机器人允许的伺服轴数
#define MAX_INFO  100//return_message支持的最大长度

#define DI_VEC "di_vec"
#define DO_VEC "do_vec"
#define AI_VEC "ai_vec"
#define AO_VEC "ao_vec"
#define CMD_VEC "cmd_vec"

constexpr double PI = 3.14159265;
constexpr double deg2rad = PI/180;
constexpr double rad2deg = 180 / PI;

typedef std::size_t Size;

typedef unsigned int        UINT;
typedef unsigned long       DWORD;
#endif
