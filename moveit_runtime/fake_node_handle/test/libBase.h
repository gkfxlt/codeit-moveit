
#pragma once

#ifdef WIN32
#define WIN_EXPORT __declspec(dllexport)
#else
#define WIN_EXPORT
#endif

class LibBase
{
};
