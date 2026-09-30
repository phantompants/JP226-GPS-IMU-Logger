#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace telemetry {

constexpr std::uint32_t kMagic = 0x4A503232U;  // "JP22"
constexpr std::uint8_t kProtocolVersion = 1;
constexpr std::size_t kEspNowV1PayloadLimit = 250;

enum class PacketType : std::uint8_t {
  Telemetry = 1,
  Discovery = 2,
  RawImuBatch = 3,
  DialCommand = 4,
  DialStatus = 5,
  DialAck = 6,
};

enum class DialAction : std::uint8_t {
  Heartbeat = 0,
  MarkWaypoint = 1,
  SetRoad = 2,
  SetTyreFront = 3,
  SetTyreRear = 4,
  SetSuspensionFront = 5,
  SetSuspensionRear = 6,
  SetLoad = 7,
};

enum StatusFlag : std::uint16_t {
  GpsFixValid = 1U << 0,
  GpsPositionValid = 1U << 1,
  GpsAltitudeValid = 1U << 2,
  GpsSpeedValid = 1U << 3,
  GpsCourseValid = 1U << 4,
  GpsSatellitesValid = 1U << 5,
  GpsHdopValid = 1U << 6,
  GpsUtcValid = 1U << 7,
  ImuValid = 1U << 8,
  ImuCalibrated = 1U << 9,
  ImuStatisticsValid = 1U << 10,
};

#pragma pack(push, 1)

struct PacketHeader {
  std::uint32_t magic;
  std::uint8_t version;
  std::uint8_t type;
  std::uint16_t size;
};

struct TelemetryPacket {
  PacketHeader header;
  std::uint32_t sequence;
  std::uint32_t uptime_ms;
  std::uint32_t sample_time_ms;
  std::uint64_t gps_utc_ms;
  std::uint16_t status_flags;
  std::uint16_t gps_age_ms;
  double latitude_deg;
  double longitude_deg;
  float altitude_m;
  float speed_kmh;
  float course_deg;
  float hdop;
  std::uint8_t satellites;
  std::uint8_t reserved0[3];
  float accel_x_mps2;
  float accel_y_mps2;
  float accel_z_mps2;
  float gyro_x_dps;
  float gyro_y_dps;
  float gyro_z_dps;
  float pitch_deg;
  float roll_deg;
  float accel_rms_mps2;
  float vertical_accel_rms_mps2;
  float vertical_accel_peak_pos_mps2;
  float vertical_accel_peak_neg_mps2;
  float lateral_accel_peak_mps2;
  float longitudinal_accel_peak_mps2;
  float vibration_rms_mps2;
  std::uint16_t imu_sample_count;
  std::uint16_t transmit_failures;
  std::uint32_t crc32;
};

struct DiscoveryPacket {
  PacketHeader header;
  std::uint32_t sequence;
  std::uint32_t uptime_ms;
  std::uint32_t crc32;
};

struct DialCommandPacket {
  PacketHeader header;
  std::uint32_t sequence;
  std::uint8_t action;
  std::uint8_t reserved0[3];
  float value;
  char text[32];
  std::uint32_t crc32;
};

struct DialStatusPacket {
  PacketHeader header;
  std::uint32_t sequence;
  std::uint32_t uptime_ms;
  std::uint32_t log_elapsed_s;
  float speed_kmh;
  float front_psi;
  float rear_psi;
  std::uint8_t fix_valid;
  std::uint8_t satellites;
  std::uint8_t log_mode;
  std::uint8_t source;
  char road[16];
  char suspension_front[12];
  char suspension_rear[12];
  char vehicle_load[12];
  char place[32];
  char last_waypoint[24];
  std::uint32_t crc32;
};

struct DialAckPacket {
  PacketHeader header;
  std::uint32_t command_sequence;
  std::uint8_t accepted;
  std::uint8_t reserved0[3];
  char waypoint_id[24];
  std::uint32_t crc32;
};

// Optional raw stream. Acceleration is milli-g, gyro is 0.1 degree/second,
// and time_offset_100us is in 100-microsecond ticks from batch_start_time_ms.
struct RawImuSample {
  std::int16_t accel_x_mg;
  std::int16_t accel_y_mg;
  std::int16_t accel_z_mg;
  std::int16_t gyro_x_deci_dps;
  std::int16_t gyro_y_deci_dps;
  std::int16_t gyro_z_deci_dps;
  std::uint16_t time_offset_100us;
};

constexpr std::size_t kRawImuSamplesPerPacket = 16;

struct RawImuBatchPacket {
  PacketHeader header;
  std::uint32_t sequence;
  std::uint32_t batch_start_time_ms;
  std::uint8_t sample_count;
  std::uint8_t reserved0[3];
  RawImuSample samples[kRawImuSamplesPerPacket];
  std::uint32_t crc32;
};

#pragma pack(pop)

static_assert(sizeof(TelemetryPacket) <= kEspNowV1PayloadLimit,
              "TelemetryPacket must fit an ESP-NOW v1 payload");
static_assert(sizeof(RawImuBatchPacket) <= kEspNowV1PayloadLimit,
              "RawImuBatchPacket must fit an ESP-NOW v1 payload");
static_assert(sizeof(DialCommandPacket) <= kEspNowV1PayloadLimit &&
                  sizeof(DialStatusPacket) <= kEspNowV1PayloadLimit &&
                  sizeof(DialAckPacket) <= kEspNowV1PayloadLimit,
              "Dial packets must fit an ESP-NOW v1 payload");
static_assert(sizeof(float) == 4 && sizeof(double) == 8,
              "Protocol requires IEEE-754 32/64-bit floating point");

inline std::uint32_t crc32(const void* data, std::size_t length) {
  const auto* bytes = static_cast<const std::uint8_t*>(data);
  std::uint32_t crc = 0xFFFFFFFFU;
  for (std::size_t i = 0; i < length; ++i) {
    crc ^= bytes[i];
    for (std::uint8_t bit = 0; bit < 8; ++bit) {
      const std::uint32_t mask = 0U - (crc & 1U);
      crc = (crc >> 1U) ^ (0xEDB88320U & mask);
    }
  }
  return ~crc;
}

template <typename Packet>
inline void preparePacket(Packet& packet, PacketType type) {
  std::memset(&packet, 0, sizeof(packet));
  packet.header.magic = kMagic;
  packet.header.version = kProtocolVersion;
  packet.header.type = static_cast<std::uint8_t>(type);
  packet.header.size = static_cast<std::uint16_t>(sizeof(Packet));
}

template <typename Packet>
inline void sealPacket(Packet& packet) {
  packet.crc32 = 0;
  packet.crc32 = crc32(&packet, sizeof(Packet) - sizeof(packet.crc32));
}

template <typename Packet>
inline bool validatePacket(const Packet& packet, PacketType type) {
  return packet.header.magic == kMagic &&
         packet.header.version == kProtocolVersion &&
         packet.header.type == static_cast<std::uint8_t>(type) &&
         packet.header.size == sizeof(Packet) &&
         packet.crc32 == crc32(&packet, sizeof(Packet) - sizeof(packet.crc32));
}

inline bool macIsUnset(const std::uint8_t mac[6]) {
  std::uint8_t combined = 0;
  for (std::size_t i = 0; i < 6; ++i) combined |= mac[i];
  return combined == 0;
}

}  // namespace telemetry
