// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// Copyright(c) 2026: PalindromicBreadLoaf (palindromicbreadloaf@tuta.com)
// SPDX-License-Identifier: GPL-3.0+

#pragma once

namespace HorizonThreadWrap
{
	// Logs to exit.txt every thread that was never joined and is still running or still has its stack
	// mirrored. With stop_running, also waits briefly for running threads, pauses the ones left and unmaps
	// their stacks so the next program started by the loader gets the heap back. Returns how many were
	// still running.
	int ReleaseLiveThreads(const char* when, bool stop_running);
} // namespace HorizonThreadWrap
