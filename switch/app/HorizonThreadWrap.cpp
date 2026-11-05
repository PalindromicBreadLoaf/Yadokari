// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// Copyright(c) 2026: PalindromicBreadLoaf (palindromicbreadloaf@tuta.com)
// SPDX-License-Identifier: GPL-3.0+

#include "HorizonThreadWrap.h"

#include "common/Horizon/Horizon.h"
#include "common/Horizon/HorizonProfiling.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <new>
#include <vector>

#include <pthread.h>

extern "C" int __real_pthread_create(pthread_t* thread, const pthread_attr_t* attr, void* (*start)(void*), void* arg);
extern "C" int __real_pthread_join(pthread_t thread, void** value);

namespace
{
	struct ThreadStart
	{
		void* (*start)(void*);
		void* arg;
		std::array<uptr, 4> arg_words;
	};

	struct LiveThread
	{
		pthread_t pthread;
		Handle handle;
		uptr stack_mem;
		size_t stack_sz;
		uptr start;
		std::array<uptr, 4> arg_words;
	};

	std::mutex s_live_mutex;
	std::vector<LiveThread> s_live_threads;

	void* ClampedThreadStart(void* param)
	{
		const ThreadStart ts = *static_cast<ThreadStart*>(param);
		delete static_cast<ThreadStart*>(param);
		Horizon::ReserveCore3ForCallingThread();
		if (const Thread* const self = threadGetSelf())
		{
			std::lock_guard lock(s_live_mutex);
			s_live_threads.push_back({pthread_self(), self->handle, reinterpret_cast<uptr>(self->stack_mem),
				self->stack_sz, reinterpret_cast<uptr>(ts.start), ts.arg_words});
		}
		return ts.start(ts.arg);
	}

	bool ArgWordsReadable(const void* arg)
	{
		const uptr addr = reinterpret_cast<uptr>(arg);
		MemoryInfo info{};
		u32 page_info;
		return addr != 0 && (addr & 7) == 0 && R_SUCCEEDED(svcQueryMemory(&info, &page_info, addr)) &&
			   (info.perm & Perm_R) && addr + sizeof(ThreadStart::arg_words) <= info.addr + info.size;
	}

	bool StackStillBorrowed(uptr stack_mem)
	{
		MemoryInfo info{};
		u32 page_info;
		return R_SUCCEEDED(svcQueryMemory(&info, &page_info, stack_mem)) && info.attr != 0;
	}
} // namespace

void HorizonThreadWrap::AuditUnjoinedThreads(const char* when)
{
	MemoryInfo text{};
	u32 page_info;
	if (R_FAILED(svcQueryMemory(&text, &page_info, reinterpret_cast<uptr>(&ClampedThreadStart))))
		text = {};
	const uptr base = text.addr;

	std::array<Horizon::Profiling::ThreadInfo, Horizon::Profiling::MAX_THREADS> names;
	const size_t name_count = Horizon::Profiling::SnapshotThreads(names);

	std::lock_guard lock(s_live_mutex);
	char line[512];
	u32 listed = 0;
	for (const LiveThread& t : s_live_threads)
	{
		if (!StackStillBorrowed(t.stack_mem))
			continue;
		const char* name = "?";
		for (size_t i = 0; i < name_count; i++)
		{
			if (names[i].handle == t.handle)
				name = names[i].name.data();
		}
		const bool exited = R_SUCCEEDED(svcWaitSynchronizationSingle(t.handle, 0));
		std::snprintf(line, sizeof(line),
			"unjoined thread (%s): '%s' %s, stack %#lx-%#lx (%#lx), module base %#lx, start +%#lx, "
			"arg words %#lx %#lx %#lx %#lx",
			when, name, exited ? "exited" : "running", t.stack_mem, t.stack_mem + t.stack_sz, t.stack_sz, base,
			t.start - base, t.arg_words[0], t.arg_words[1], t.arg_words[2], t.arg_words[3]);
		Horizon::ExitTrace(line);
		listed++;
	}
	std::snprintf(line, sizeof(line), "unjoined thread (%s): %u listed", when, listed);
	Horizon::ExitTrace(line);
}

extern "C" int __wrap_pthread_create(pthread_t* thread, const pthread_attr_t* attr, void* (*start)(void*), void* arg)
{
	ThreadStart* const ts = new (std::nothrow) ThreadStart{start, arg, {}};
	if (!ts)
		return EAGAIN;
	if (ArgWordsReadable(arg))
		std::memcpy(ts->arg_words.data(), arg, sizeof(ts->arg_words));
	const int result = __real_pthread_create(thread, attr, ClampedThreadStart, ts);
	if (result != 0)
		delete ts;
	return result;
}

extern "C" int __wrap_pthread_join(pthread_t thread, void** value)
{
	const int result = __real_pthread_join(thread, value);
	if (result == 0)
	{
		std::lock_guard lock(s_live_mutex);
		std::erase_if(s_live_threads, [thread](const LiveThread& t) { return t.pthread == thread; });
	}
	return result;
}
