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
		uptr stack_mirror;
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
				reinterpret_cast<uptr>(self->stack_mirror), self->stack_sz, reinterpret_cast<uptr>(ts.start),
				ts.arg_words});
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

	// A thread's stack is mirrored with svcMapMemory: the heap pages under it are inaccessible until the
	// mirror is unmapped (threadClose). Returns the mapping if the mirror is still there.
	bool MirrorStillMapped(uptr mirror, MemoryInfo& info)
	{
		u32 page_info;
		return mirror != 0 && R_SUCCEEDED(svcQueryMemory(&info, &page_info, mirror)) &&
			   (info.type & 0xff) == MemType_MappedMemory && info.addr == mirror;
	}

	bool ThreadHasExited(Handle handle)
	{
		return R_SUCCEEDED(svcWaitSynchronizationSingle(handle, 0));
	}
} // namespace

int HorizonThreadWrap::ReleaseLiveThreads(const char* when, bool stop_running)
{
	MemoryInfo text{};
	u32 page_info;
	if (R_FAILED(svcQueryMemory(&text, &page_info, reinterpret_cast<uptr>(&ClampedThreadStart))))
		text = {};
	const uptr base = text.addr;

	std::array<Horizon::Profiling::ThreadInfo, Horizon::Profiling::MAX_THREADS> names;
	const size_t name_count = Horizon::Profiling::SnapshotThreads(names);

	std::vector<LiveThread> threads;
	{
		std::lock_guard lock(s_live_mutex);
		threads = s_live_threads;
	}

	char line[512];
	if (stop_running)
	{
		// Threads still running get a moment to end by themselves.
		for (int waited = 0; waited < 1000; waited += 10)
		{
			bool any_running = false;
			for (const LiveThread& t : threads)
				any_running |= !ThreadHasExited(t.handle);
			if (!any_running)
				break;
			svcSleepThread(10'000'000ULL);
		}
	}

	u32 listed = 0, running = 0, released = 0;
	for (const LiveThread& t : threads)
	{
		const bool exited = ThreadHasExited(t.handle);
		const char* name = "?";
		for (size_t i = 0; i < name_count; i++)
		{
			if (names[i].handle == t.handle)
				name = names[i].name.data();
		}

		MemoryInfo info{};
		const bool mapped = MirrorStillMapped(t.stack_mirror, info);
		if (exited && !mapped)
			continue;

		listed++;
		if (!exited)
			running++;
		std::snprintf(line, sizeof(line),
			"unjoined thread (%s): '%s' %s, mirror %s, stack %#lx (%#lx), module base %#lx, start +%#lx, "
			"arg words %#lx %#lx %#lx %#lx",
			when, name, exited ? "exited" : "RUNNING", mapped ? "mapped" : "gone", t.stack_mem, t.stack_sz, base,
			t.start - base, t.arg_words[0], t.arg_words[1], t.arg_words[2], t.arg_words[3]);
		Horizon::ExitTrace(line);

		if (!stop_running)
			continue;

		// A thread left running keeps executing this program's code after it has exited (the next program
		// runs in this same process), and its stack keeps heap pages locked. Pause it for good, as Kaizhar does.
		if (!exited && R_FAILED(svcSetThreadActivity(t.handle, ThreadActivity_Paused)))
			continue;
		if (mapped && R_SUCCEEDED(svcUnmapMemory(reinterpret_cast<void*>(t.stack_mirror),
							   reinterpret_cast<void*>(t.stack_mem), info.size)))
			released++;
	}
	std::snprintf(line, sizeof(line), "unjoined thread (%s): %u listed, %u running, %u stacks released", when, listed,
		running, released);
	Horizon::ExitTrace(line);
	return static_cast<int>(running);
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
