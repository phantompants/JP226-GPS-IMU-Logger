#pragma once

#include <cstdint>

namespace schedule {

// Returns the first scheduled stopped-log epoch strictly after `afterUtc`.
// The sequence is stop+15m, +30m, +45m, +60m, then +2h, +3h, ...
std::int64_t nextStoppedDue(std::int64_t stopUtc, std::int64_t afterUtc,
                            std::uint32_t firstHourIntervalSec,
                            std::uint32_t hourlyIntervalSec);

// True only for a plausible saved stopped session. This prevents corrupt or
// very old NVS values from suppressing a fresh stopped-session record.
bool persistedStopIsPlausible(std::int64_t stopUtc, std::int64_t nextDueUtc,
                              std::int64_t nowUtc,
                              std::uint32_t maximumAgeSec,
                              std::uint32_t maximumFutureDueSec);

}  // namespace schedule

