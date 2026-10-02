#pragma once

// Original pixel-art brontosaurus for the "Brought to you by JP226Prints"
// page on every device with a screen. Two running frames, 24x16 pixels:
// G body, D back plates, W eye, K pupil, '.' transparent.

#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace dino {

constexpr int kWidth = 24;
constexpr int kHeight = 16;
constexpr int kFrameCount = 2;

constexpr const char* kFrames[kFrameCount][kHeight] = {
    {
        "..................GGGG..",
        ".................GGGWKG.",
        ".................GGGGGGG",
        ".................GGG....",
        "................GGG.....",
        "...............GGG......",
        ".......D.D.D..GGG.......",
        ".....GGGGGGGGGGGG.......",
        "...GGGDGGGGDGGGGG.......",
        "..GGGGGGGGGGGGGGG.......",
        ".GG.GGGGGGGGGGGGG.......",
        "GG...GGGGGGGGGGG........",
        ".....GG..GG..GG..GG.....",
        ".....GG..GG..GG..GG.....",
        "....GGG.GGG.GGG.GGG.....",
        "........................",
    },
    {
        "..................GGGG..",
        ".................GGGWKG.",
        ".................GGGGGGG",
        ".................GGG....",
        "................GGG.....",
        "...............GGG......",
        ".......D.D.D..GGG.......",
        ".....GGGGGGGGGGGG.......",
        "...GGGDGGGGDGGGGG.......",
        "..GGGGGGGGGGGGGGG.......",
        ".GG.GGGGGGGGGGGGG.......",
        "GG...GGGGGGGGGGG........",
        "......GG.GG....GG.GG....",
        "......GG.GG....GG.GG....",
        ".....GGGGGG...GGGGGG....",
        "........................",
    },
};

constexpr uint16_t kBody = 0x3DA9;   // RGB565 of (60, 180, 75)
constexpr uint16_t kPlates = 0x1B65; // RGB565 of (30, 110, 40)
constexpr uint16_t kEye = 0xFFFF;
constexpr uint16_t kPupil = 0x0000;

// Draws one frame with its top-left at (x, y), each sprite pixel `scale`
// screen pixels square. Works with any M5GFX display or canvas.
template <typename Gfx>
void draw(Gfx& gfx, int x, int y, int scale, int frame) {
  const char* const* rows = kFrames[frame % kFrameCount];
  for (int row = 0; row < kHeight; ++row) {
    for (int col = 0; col < kWidth; ++col) {
      uint16_t color;
      switch (rows[row][col]) {
        case 'G': color = kBody; break;
        case 'D': color = kPlates; break;
        case 'W': color = kEye; break;
        case 'K': color = kPupil; break;
        default: continue;
      }
      gfx.fillRect(x + col * scale, y + row * scale, scale, scale, color);
    }
  }
}

// Frame and horizontal position for a dinosaur running left to right across
// a strip `areaWidth` pixels wide, re-entering from the left.
inline int frameAt(uint32_t nowMs) { return (nowMs / 150) % kFrameCount; }
inline int runX(uint32_t nowMs, int areaWidth, int scale) {
  const int span = areaWidth + kWidth * scale;
  return static_cast<int>((nowMs / 25) % span) - kWidth * scale;
}

constexpr const char kCredit1[] = "Brought to you by";
constexpr const char kCredit2[] = "JP226Prints";
// Preferences key for the personal best, in seconds. (Older firmware kept an
// obstacle count under "dino_best"; a new key keeps those out of the PBs.)
constexpr const char kBestKey[] = "dino_pb";

// "Dino Dash": while the vehicle is moving the dinosaur just runs across the
// screen; once parked it becomes a game. Rocks and cacti scroll in from the
// right and a press makes the dinosaur jump. Positions are in sprite pixels
// ("world units"), so every screen plays the same game at its own scale.
class Game {
 public:
  enum class State : uint8_t { Auto, Ready, Playing, Over };

  // Above kMovingKmh it is driving; it only counts as parked again below
  // kParkedKmh, so the mode does not flicker around one speed.
  static constexpr float kMovingKmh = 5.0f;
  static constexpr float kParkedKmh = 3.0f;

  // Call every frame with the vehicle speed (negative when unknown, which
  // counts as parked) and the play area width in world units.
  void update(uint32_t nowMs, float speedKmh, int worldWidth) {
    const uint32_t elapsed = nowMs - lastMs_;
    lastMs_ = nowMs;
    const float dt = (elapsed > 100 ? 100 : elapsed) / 1000.0f;
    worldWidth_ = worldWidth;
    if (speedKmh > kMovingKmh) moving_ = true;
    else if (speedKmh < kParkedKmh) moving_ = false;
    if (moving_) {
      state_ = State::Auto;
      return;
    }
    if (state_ == State::Auto) {
      state_ = State::Ready;
      reset();
    }
    if (state_ != State::Playing) return;

    // Starts gently and speeds up slowly.
    // The score is whole seconds survived. Starts gently and speeds up as
    // the run goes on.
    score_ = static_cast<uint16_t>((nowMs - startMs_) / 1000);
    const float speed = score_ * 0.7f + 32.0f < 90.0f ? score_ * 0.7f + 32.0f
                                                      : 90.0f;
    const float previousY = y_;
    // Rocks and cacti are platforms: the dinosaur can land on top and ride
    // along, and only crashes into the side of one. The floor is the highest
    // top it is standing over.
    float floor = 0.0f;
    for (Obstacle& obstacle : obstacles_) {
      obstacle.x -= speed * dt;
      if (obstacle.x + obstacle.width < 0.0f) respawn(obstacle);
      // Only the legs collide, with a forgiving unit trimmed off each side.
      const bool overlap = obstacle.x < kDinoX + 18 &&
                           obstacle.x + obstacle.width > kDinoX + 6;
      if (!overlap) continue;
      if (previousY >= obstacle.height - 1.0f) {
        if (obstacle.height > floor) floor = obstacle.height;
      } else {
        crash(nowMs);
        return;
      }
    }
    vy_ -= kGravity * dt;
    y_ += vy_ * dt;
    onGround_ = y_ <= floor;
    if (onGround_) {
      y_ = floor;
      vy_ = 0.0f;
    }
  }

  // Jump, or start / restart a game. Ignored while moving.
  void press(uint32_t nowMs) {
    switch (state_) {
      case State::Ready:
        reset();
        state_ = State::Playing;
        vy_ = kJumpSpeed;
        break;
      case State::Playing:
        if (onGround_) {
          vy_ = kJumpSpeed;
          onGround_ = false;
        }
        break;
      case State::Over:
        // A short pause so a late jump does not restart straight away.
        if (nowMs - overMs_ > 700) {
          reset();
          state_ = State::Playing;
        }
        break;
      default:
        break;
    }
  }

  State state() const { return state_; }
  uint16_t score() const { return score_; }
  // True on the game-over screen when that run set a new personal best.
  bool beatBest() const { return beatBest_; }

  // "12s  PB 34s" while playing, "GAME OVER 12s  PB 34s" or "NEW PB 41s!"
  // afterwards; empty otherwise. Screens add their own "press to play".
  void statusText(char* text, size_t size) const {
    text[0] = '\0';
    if (state_ == State::Playing) {
      snprintf(text, size, "%us  PB %us", score_, best_);
    } else if (state_ == State::Over && beatBest_) {
      snprintf(text, size, "NEW PB %us!", score_);
    } else if (state_ == State::Over) {
      snprintf(text, size, "GAME OVER %us  PB %us", score_, best_);
    }
  }
  uint16_t best() const { return best_; }
  void setBest(uint16_t best) { best_ = best; }
  // True once after a game sets a new best, so the caller can save it.
  bool takeNewBest() {
    const bool result = newBest_;
    newBest_ = false;
    return result;
  }

  // Draws the play area: `left` and `widthPx` on screen, ground at `groundY`.
  template <typename Gfx>
  void draw(Gfx& gfx, int left, int groundY, int widthPx, int scale,
            uint32_t nowMs) const {
    const int top = groundY - kHeight * scale;
    if (state_ == State::Auto) {
      dino::draw(gfx, left + runX(nowMs, widthPx, scale), top, scale,
                 frameAt(nowMs));
      for (int x = -static_cast<int>((nowMs / 25) % (4 * scale));
           x < widthPx; x += 4 * scale) {
        gfx.drawFastHLine(left + x, groundY, 2 * scale, 0x7BEF);
      }
      return;
    }
    gfx.drawFastHLine(left, groundY, widthPx, 0x7BEF);
    for (const Obstacle& obstacle : obstacles_) {
      const int x = left + static_cast<int>(obstacle.x * scale);
      if (x >= left + widthPx || x + obstacle.width * scale <= left) continue;
      drawObstacle(gfx, obstacle, x, groundY, scale);
    }
    const bool running = state_ == State::Playing && onGround_;
    dino::draw(gfx, left + static_cast<int>(kDinoX * scale),
               top - static_cast<int>(y_ * scale), scale,
               running ? frameAt(nowMs) : 0);
  }

 private:
  struct Obstacle {
    float x = 0.0f;
    uint8_t width = 0;
    uint8_t height = 0;
    bool cactus = false;
  };

  static constexpr int kDinoX = 4;
  static constexpr float kGravity = 220.0f;  // world units per second squared
  static constexpr float kJumpSpeed = 95.0f;  // about 20 units high
  static constexpr int kObstacleCount = 3;
  static constexpr uint16_t kCactus = 0x2D07;  // RGB565 of (40, 160, 60)
  static constexpr uint16_t kRock = 0x8410;

  uint32_t nextRandom() {
    seed_ ^= seed_ << 13;
    seed_ ^= seed_ >> 17;
    seed_ ^= seed_ << 5;
    return seed_;
  }

  void respawn(Obstacle& obstacle) {
    float rightmost = static_cast<float>(worldWidth_);
    for (const Obstacle& other : obstacles_) {
      if (other.x + other.width > rightmost) rightmost = other.x + other.width;
    }
    obstacle.cactus = nextRandom() % 2 == 0;
    obstacle.width = obstacle.cactus ? 6 : 7;
    obstacle.height = obstacle.cactus ? 9 : 4 + nextRandom() % 3;
    // Gaps long enough to land and jump again at the fastest speed.
    obstacle.x = rightmost + 50 + nextRandom() % 50;
  }

  void crash(uint32_t nowMs) {
    state_ = State::Over;
    overMs_ = nowMs;
    beatBest_ = score_ > best_;
    if (beatBest_) {
      best_ = score_;
      newBest_ = true;
    }
  }

  void reset() {
    seed_ ^= lastMs_ | 1U;
    y_ = 0.0f;
    vy_ = 0.0f;
    onGround_ = true;
    score_ = 0;
    startMs_ = lastMs_;
    beatBest_ = false;
    for (Obstacle& obstacle : obstacles_) obstacle.x = -100.0f;
    for (Obstacle& obstacle : obstacles_) respawn(obstacle);
  }

  template <typename Gfx>
  static void drawObstacle(Gfx& gfx, const Obstacle& obstacle, int x,
                           int groundY, int scale) {
    const int h = obstacle.height;
    if (!obstacle.cactus) {
      gfx.fillRoundRect(x, groundY - h * scale, obstacle.width * scale,
                        h * scale, scale, kRock);
      return;
    }
    // Trunk two units wide, an arm on each side.
    gfx.fillRect(x + 2 * scale, groundY - h * scale, 2 * scale, h * scale,
                 kCactus);
    gfx.fillRect(x, groundY - 6 * scale, 2 * scale, scale, kCactus);
    gfx.fillRect(x, groundY - 8 * scale, scale, 3 * scale, kCactus);
    gfx.fillRect(x + 4 * scale, groundY - 4 * scale, 2 * scale, scale,
                 kCactus);
    gfx.fillRect(x + 5 * scale, groundY - 6 * scale, scale, 3 * scale,
                 kCactus);
  }

  State state_ = State::Auto;
  bool moving_ = true;
  bool newBest_ = false;
  bool beatBest_ = false;
  bool onGround_ = true;
  float y_ = 0.0f;
  float vy_ = 0.0f;
  uint16_t score_ = 0;
  uint16_t best_ = 0;
  uint32_t lastMs_ = 0;
  uint32_t overMs_ = 0;
  uint32_t startMs_ = 0;
  uint32_t seed_ = 0x2545F491U;
  int worldWidth_ = 80;
  Obstacle obstacles_[kObstacleCount]{};
};

}  // namespace dino
