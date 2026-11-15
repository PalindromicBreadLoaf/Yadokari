// SPDX-FileCopyrightText: 2026 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

// Switch 2 (kaiHEN): checks each system service lookup before it reaches
// kaiHEN, and logs it.
//
// Under kaiHEN, a program's service lookups go to HEN's SM proxy. The proxy
// forwards every program's GetServiceHandle over one SM session that HEN keeps
// for the whole boot, registered as FS. SM does not fail a lookup for a name
// that is not registered: it defers the reply until the service appears. A
// deferred request holds that session, so one lookup of a service that does
// not exist on this firmware would stop every later lookup through HEN until
// the console reboots. The next hbloader launch (holding R) would then never
// get its services: a black screen.

#ifdef __SWITCH2__

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/stat.h>

#define u128 libnx_u128
#define s128 libnx_s128
#include <switch.h>
#undef u128
#undef s128

#include "common/Horizon/Horizon.h"

extern "C" Result __real_smGetServiceWrapper(Service* service_out, SmServiceName name);
extern "C" Result __real_smGetServiceOriginal(Handle* handle_out, SmServiceName name);

namespace
{
	constexpr Result SM_RESULT_NOT_ALLOWED = MAKERESULT(21, 8);
	constexpr u64 PROBE_TIMEOUT_NS = 500'000'000;
	constexpr const char* ALLOW_UNVERIFIED_FILE = "sdmc:/switch/yadokari/diag/sm-allow-unverified";

	struct Probe
	{
		Thread thread;
		SmServiceName name;
		Handle session;
		Handle result_handle;
		Result result;
		bool in_use;
		bool stuck;
	};

	constexpr int MAX_PROBES = 16;
	constexpr int MAX_MISSING = 32;
	constexpr int MAX_EARLY_LINES = 64;

	// All plain zero-initialized data: lookups start in libnx's __appInit, before
	// any C++ constructor has run.
	Mutex s_lock;
	Probe s_probes[MAX_PROBES];
	SmServiceName s_missing[MAX_MISSING];
	int s_missing_count;
	bool s_after_startup;
	bool s_allow_unverified;
	char s_early[MAX_EARLY_LINES][96];
	int s_early_count;
	int s_early_dropped;

	void NameToString(SmServiceName name, char (&out)[9])
	{
		std::memcpy(out, name.name, 8);
		out[8] = '\0';
	}

	void Log(const char* format, ...) __attribute__((format(printf, 1, 2)));
	void Log(const char* format, ...)
	{
		char line[96];
		va_list args;
		va_start(args, format);
		std::vsnprintf(line, sizeof(line), format, args);
		va_end(args);
		if (s_after_startup)
		{
			Horizon::ExitTrace(line);
			return;
		}
		if (s_early_count < MAX_EARLY_LINES)
			std::memcpy(s_early[s_early_count++], line, sizeof(line));
		else
			++s_early_dropped;
	}

	bool IsKnownMissing(SmServiceName name)
	{
		for (int i = 0; i < s_missing_count; ++i)
		{
			if (smServiceNamesAreEqual(s_missing[i], name))
				return true;
		}
		return false;
	}

	void ProbeMain(void* argument)
	{
		Probe* const probe = static_cast<Probe*>(argument);
		Handle session = INVALID_HANDLE;
		Result rc = svcConnectToNamedPort(&session, "sm:");
		if (R_SUCCEEDED(rc))
		{
			probe->session = session;
			Service sm;
			serviceCreate(&sm, session);
			u64 pid_placeholder = 0;
			rc = serviceDispatchIn(&sm, 0, pid_placeholder, .in_send_pid = true); // RegisterClient
			if (R_SUCCEEDED(rc))
			{
				Handle handle = INVALID_HANDLE;
				rc = serviceDispatchIn(&sm, 1, probe->name, // GetServiceHandle
					.out_handle_attrs = {SfOutHandleAttr_HipcMove},
					.out_handles = &handle);
				if (R_SUCCEEDED(rc))
					probe->result_handle = handle;
			}
		}
		probe->result = rc;
	}

	enum class ProbeOutcome
	{
		Found,
		Missing,
		Refused,
		Unavailable,
	};

	ProbeOutcome RunProbe(SmServiceName name, Handle* out_handle, Result* out_result)
	{
		Probe* probe = nullptr;
		for (Probe& candidate : s_probes)
		{
			if (!candidate.in_use)
			{
				probe = &candidate;
				break;
			}
		}
		if (!probe)
			return ProbeOutcome::Unavailable;

		*probe = {};
		probe->in_use = true;
		probe->name = name;
		probe->session = INVALID_HANDLE;
		probe->result_handle = INVALID_HANDLE;
		if (R_FAILED(threadCreate(&probe->thread, ProbeMain, probe, nullptr, 0x4000, 0x2C, -2)))
		{
			probe->in_use = false;
			return ProbeOutcome::Unavailable;
		}
		if (R_FAILED(threadStart(&probe->thread)))
		{
			threadClose(&probe->thread);
			probe->in_use = false;
			return ProbeOutcome::Unavailable;
		}

		const Result wait = svcWaitSynchronizationSingle(probe->thread.handle, PROBE_TIMEOUT_NS);
		if (R_FAILED(wait))
		{
			// SM deferred the reply: the name is not registered. The thread and its
			// session stay as they are; they are paused and closed at exit.
			probe->stuck = true;
			return ProbeOutcome::Missing;
		}

		threadWaitForExit(&probe->thread);
		threadClose(&probe->thread);
		if (probe->session != INVALID_HANDLE)
			svcCloseHandle(probe->session);
		const Result rc = probe->result;
		*out_handle = probe->result_handle;
		probe->in_use = false;
		*out_result = rc;
		if (R_SUCCEEDED(rc))
			return ProbeOutcome::Found;
		if (rc == SM_RESULT_NOT_ALLOWED)
			return ProbeOutcome::Refused;
		return ProbeOutcome::Unavailable;
	}

	bool GuardedLookup(SmServiceName name, Handle* out_handle, Result* out_rc)
	{
		char text[9];
		NameToString(name, text);

		mutexLock(&s_lock);
		if (IsKnownMissing(name))
		{
			mutexUnlock(&s_lock);
			Log("sm: %s: refused again (missing)", text);
			*out_rc = SM_RESULT_NOT_ALLOWED;
			return true;
		}

		Handle handle = INVALID_HANDLE;
		Result probe_rc = 0;
		const ProbeOutcome outcome = RunProbe(name, &handle, &probe_rc);
		if (outcome == ProbeOutcome::Missing && s_missing_count < MAX_MISSING)
			s_missing[s_missing_count++] = name;
		const bool after_startup = s_after_startup;
		mutexUnlock(&s_lock);

		switch (outcome)
		{
			case ProbeOutcome::Found:
				Log("sm: %s: found (own session)", text);
				*out_handle = handle;
				*out_rc = 0;
				return true;
			case ProbeOutcome::Missing:
				Log("sm: %s: MISSING, no reply in %lu ms; refused so HEN does not wait for it", text,
					static_cast<unsigned long>(PROBE_TIMEOUT_NS / 1'000'000));
				*out_rc = SM_RESULT_NOT_ALLOWED;
				return true;
			case ProbeOutcome::Refused:
				if (!after_startup || s_allow_unverified)
				{
					Log("sm: %s: not allowed for this process; asking HEN (%s)", text,
						after_startup ? "sm-allow-unverified" : "start-up service");
					return false;
				}
				Log("sm: %s: not allowed for this process and unverified; refused", text);
				*out_rc = SM_RESULT_NOT_ALLOWED;
				return true;
			case ProbeOutcome::Unavailable:
			default:
				Log("sm: %s: check failed (0x%x); asking HEN", text, probe_rc);
				return false;
		}
	}

	__attribute__((constructor(103))) void FlushEarlyLookups()
	{
		struct stat info{};
		s_allow_unverified = stat(ALLOW_UNVERIFIED_FILE, &info) == 0;
		mutexLock(&s_lock);
		for (int i = 0; i < s_early_count; ++i)
		{
			char line[112];
			std::snprintf(line, sizeof(line), "%s (before main)", s_early[i]);
			Horizon::ExitTrace(line);
		}
		if (s_early_dropped)
		{
			char line[64];
			std::snprintf(line, sizeof(line), "sm: %d earlier lookup line(s) dropped", s_early_dropped);
			Horizon::ExitTrace(line);
		}
		{
			char line[96];
			std::snprintf(line, sizeof(line), "sm: lookup guard active, unverified names %s",
				s_allow_unverified ? "go to HEN" : "refused");
			Horizon::ExitTrace(line);
		}
		s_after_startup = true;
		mutexUnlock(&s_lock);
	}

	void PauseStuckProbes()
	{
		mutexLock(&s_lock);
		for (Probe& probe : s_probes)
		{
			if (!probe.in_use || !probe.stuck)
				continue;
			svcSetThreadActivity(probe.thread.handle, ThreadActivity_Paused);
			if (probe.session != INVALID_HANDLE)
				svcCloseHandle(probe.session);
			probe.session = INVALID_HANDLE;
		}
		mutexUnlock(&s_lock);
	}

	__attribute__((constructor(103))) void RegisterExit()
	{
		std::atexit(PauseStuckProbes);
	}
} // namespace

extern "C" Result __wrap_smGetServiceWrapper(Service* service_out, SmServiceName name)
{
	if (smGetServiceOverride(name) != INVALID_HANDLE)
		return __real_smGetServiceWrapper(service_out, name);

	Handle handle = INVALID_HANDLE;
	Result rc = 0;
	if (!GuardedLookup(name, &handle, &rc))
		return __real_smGetServiceWrapper(service_out, name);
	if (R_SUCCEEDED(rc))
		serviceCreate(service_out, handle);
	return rc;
}

extern "C" Result __wrap_smGetServiceOriginal(Handle* handle_out, SmServiceName name)
{
	Result rc = 0;
	if (!GuardedLookup(name, handle_out, &rc))
		return __real_smGetServiceOriginal(handle_out, name);
	return rc;
}

#endif // __SWITCH2__
