#ifndef CODEIT_CONTROLLER_NRT_TIMER_H_
#define CODEIT_CONTROLLER_NRT_TIMER_H_

#include <cstdint>
#include <any>

namespace codeit::controller
{
	auto aris_nrt_mlockall()->void;

	auto aris_nrt_task_create()->std::any;
	auto aris_nrt_task_start(std::any& handle, void(*task_func)(void*), void* param)->int;
	auto aris_nrt_task_join(std::any& handle)->int;
	auto aris_nrt_task_set_periodic(int nanoseconds)->int;
	auto aris_nrt_task_wait_period()->int;
	auto aris_nrt_timer_read()->std::int64_t;

	// in nano seconds
	auto aris_nrt_time_since_last_time()->std::int64_t;
}

#endif
