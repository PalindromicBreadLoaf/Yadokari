// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// Copyright(c) 2026: PalindromicBreadLoaf (palindromicbreadloaf@tuta.com)
// SPDX-License-Identifier: GPL-3.0+

#include "common/Threading.h"

#include <gtest/gtest.h>

#include <atomic>
#include <thread>

namespace
{
	enum class WaitKind
	{
		Spin,
		Sleep,
		Check,
	};

	void RunProducerConsumer(WaitKind kind, u32 batches, u32 max_batch)
	{
		Threading::WorkSema sema;
		std::atomic<u32> produced{0};
		std::atomic<u32> consumed{0};
		std::atomic<bool> stop{false};

		std::thread worker([&]() {
			u32 seen = 0;
			for (;;)
			{
				switch (kind)
				{
					case WaitKind::Spin:
						sema.WaitForWorkWithSpin();
						break;
					case WaitKind::Sleep:
						sema.WaitForWork();
						break;
					case WaitKind::Check:
						if (!sema.CheckForWork())
							sema.WaitForWork();
						break;
				}
				if (stop.load(std::memory_order_acquire))
					break;
				for (u32 avail = produced.load(std::memory_order_acquire); seen != avail;
					 avail = produced.load(std::memory_order_acquire))
				{
					seen = avail;
					consumed.store(seen, std::memory_order_release);
				}
			}
			sema.Kill();
		});

		u32 rng = 12345;
		for (u32 b = 0; b < batches; b++)
		{
			rng = rng * 1664525u + 1013904223u;
			const u32 count = 1 + (rng >> 16) % max_batch;
			for (u32 i = 0; i < count; i++)
			{
				produced.store(produced.load(std::memory_order_relaxed) + 1, std::memory_order_release);
				sema.NotifyOfWork();
			}
			ASSERT_TRUE(sema.WaitForEmpty());
			ASSERT_EQ(consumed.load(std::memory_order_acquire), produced.load(std::memory_order_relaxed))
				<< "lost wakeup in batch " << b;
		}

		stop.store(true, std::memory_order_release);
		sema.NotifyOfWork();
		worker.join();
	}
} // namespace

TEST(WorkSema, SpinWorkerSeesEveryNotify)
{
	RunProducerConsumer(WaitKind::Spin, 20000, 64);
}

TEST(WorkSema, SleepWorkerSeesEveryNotify)
{
	RunProducerConsumer(WaitKind::Sleep, 20000, 64);
}

TEST(WorkSema, CheckForWorkSeesEveryNotify)
{
	RunProducerConsumer(WaitKind::Check, 20000, 64);
}
