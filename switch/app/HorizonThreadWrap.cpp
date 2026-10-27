// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// Copyright(c) 2026: PalindromicBreadLoaf (palindromicbreadloaf@tuta.com)
// SPDX-License-Identifier: GPL-3.0+

#include "common/Horizon/Horizon.h"

#include <cerrno>
#include <new>

#include <pthread.h>

extern "C" int __real_pthread_create(pthread_t* thread, const pthread_attr_t* attr, void* (*start)(void*), void* arg);

namespace
{
	struct ThreadStart
	{
		void* (*start)(void*);
		void* arg;
	};

	void* ClampedThreadStart(void* param)
	{
		const ThreadStart ts = *static_cast<ThreadStart*>(param);
		delete static_cast<ThreadStart*>(param);
		Horizon::ReserveCore3ForCallingThread();
		return ts.start(ts.arg);
	}
} // namespace

extern "C" int __wrap_pthread_create(pthread_t* thread, const pthread_attr_t* attr, void* (*start)(void*), void* arg)
{
	ThreadStart* const ts = new (std::nothrow) ThreadStart{start, arg};
	if (!ts)
		return EAGAIN;
	const int result = __real_pthread_create(thread, attr, ClampedThreadStart, ts);
	if (result != 0)
		delete ts;
	return result;
}
