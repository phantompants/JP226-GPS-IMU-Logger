#pragma once

// Firmware version shown on every device's FOSSIL RECORD page; the git commit
// and build date come from shared/version.py at build time (a "+" after the
// commit means uncommitted changes were built).
//
// Test builds count up from v0.10 (v0.11, v0.12, ...). The first public
// release is v1.0, once a build has been tested on every logger including the
// Cardputer ADV. The hatched date already records when each build was made.

#ifndef JP226_GIT_HASH
#define JP226_GIT_HASH "dev"
#endif
#ifndef JP226_BUILD_DATE
#define JP226_BUILD_DATE __DATE__
#endif

namespace version {

constexpr char kNumber[] = "v0.15";
constexpr char kGit[] = JP226_GIT_HASH;
constexpr char kDate[] = JP226_BUILD_DATE;

// One-liners for the FOSSIL RECORD page, changing every few seconds. Each
// fits 21 characters, the width of the AtomS3's screen.
constexpr const char* kJokes[] = {
    "No dinos were harmed", "Small arms, big logs", "Runs on fossil fuel",
    "Rawr means logging",   "Data finds a way",     "Clever girl.",
    "Must go faster!",      "Bugs now extinct*",    "T-Rex approved",
    "Jurassic parking OK",  "Tracks since 65M BC",  "Dino-mite firmware",
};
constexpr int kJokeCount = sizeof(kJokes) / sizeof(kJokes[0]);

inline const char* jokeAt(unsigned long nowMs) {
  return kJokes[(nowMs / 4000) % kJokeCount];
}

}  // namespace version
