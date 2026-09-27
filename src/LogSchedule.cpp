#include "LogSchedule.h"

namespace schedule {

std::int64_t nextStoppedDue(std::int64_t stopUtc, std::int64_t afterUtc,
                            std::uint32_t firstHourIntervalSec,
                            std::uint32_t hourlyIntervalSec) {
  if (stopUtc <= 0 || firstHourIntervalSec == 0 || hourlyIntervalSec == 0) {
    return 0;
  }
  if (afterUtc < stopUtc) {
    return stopUtc + firstHourIntervalSec;
  }

  const std::int64_t elapsed = afterUtc - stopUtc;
  if (elapsed < static_cast<std::int64_t>(hourlyIntervalSec)) {
    const std::int64_t slot = elapsed / firstHourIntervalSec + 1;
    return stopUtc + slot * firstHourIntervalSec;
  }

  const std::int64_t hour = elapsed / hourlyIntervalSec + 1;
  return stopUtc + hour * hourlyIntervalSec;
}

bool persistedStopIsPlausible(std::int64_t stopUtc, std::int64_t nextDueUtc,
                              std::int64_t nowUtc,
                              std::uint32_t maximumAgeSec,
                              std::uint32_t maximumFutureDueSec) {
  if (stopUtc <= 0 || nextDueUtc <= stopUtc || nowUtc <= 0) {
    return false;
  }
  if (stopUtc > nowUtc + 5 * 60) {
    return false;
  }
  if (nextDueUtc > nowUtc + static_cast<std::int64_t>(maximumFutureDueSec)) {
    return false;
  }
  return nowUtc - stopUtc <= static_cast<std::int64_t>(maximumAgeSec);
}

}  // namespace schedule

