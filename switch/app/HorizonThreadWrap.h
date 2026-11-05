// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// Copyright(c) 2026: PalindromicBreadLoaf (palindromicbreadloaf@tuta.com)
// SPDX-License-Identifier: GPL-3.0+

#pragma once

namespace HorizonThreadWrap
{
	// Logs to exit.txt every thread that was never joined and whose stack is still
	// borrowed from the heap.
	void AuditUnjoinedThreads(const char* when);
} // namespace HorizonThreadWrap
