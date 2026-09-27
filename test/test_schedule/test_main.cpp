#include <cstdint>
#include <unity.h>

#include "LogSchedule.h"

void setUp() {}
void tearDown() {}

void test_stopped_schedule_boundaries() {
  constexpr std::int64_t stop = 1'000'000;
  constexpr std::uint32_t quarterHour = 900;
  constexpr std::uint32_t hour = 3600;

  TEST_ASSERT_EQUAL_INT64(stop + 900,
                          schedule::nextStoppedDue(stop, stop, quarterHour, hour));
  TEST_ASSERT_EQUAL_INT64(
      stop + 900, schedule::nextStoppedDue(stop, stop + 899, quarterHour, hour));
  TEST_ASSERT_EQUAL_INT64(
      stop + 1800, schedule::nextStoppedDue(stop, stop + 900, quarterHour, hour));
  TEST_ASSERT_EQUAL_INT64(
      stop + 3600, schedule::nextStoppedDue(stop, stop + 3599, quarterHour, hour));
  TEST_ASSERT_EQUAL_INT64(
      stop + 7200, schedule::nextStoppedDue(stop, stop + 3600, quarterHour, hour));
  TEST_ASSERT_EQUAL_INT64(
      stop + 10800, schedule::nextStoppedDue(stop, stop + 7200, quarterHour, hour));
}

void test_persisted_stop_sanity_checks() {
  constexpr std::int64_t stop = 1'000'000;
  TEST_ASSERT_TRUE(
      schedule::persistedStopIsPlausible(stop, stop + 900, stop + 1, 86400, 3600));
  TEST_ASSERT_FALSE(
      schedule::persistedStopIsPlausible(stop, stop, stop + 1, 86400, 3600));
  TEST_ASSERT_FALSE(schedule::persistedStopIsPlausible(
      stop, stop + 900, stop + 86401, 86400, 3600));
  TEST_ASSERT_FALSE(schedule::persistedStopIsPlausible(
      stop + 301, stop + 1200, stop, 86400, 3600));
  TEST_ASSERT_FALSE(schedule::persistedStopIsPlausible(
      stop, stop + 7200, stop + 1, 86400, 3600));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_stopped_schedule_boundaries);
  RUN_TEST(test_persisted_stop_sanity_checks);
  return UNITY_END();
}
