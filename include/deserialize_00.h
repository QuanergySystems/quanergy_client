#ifndef QUANERGY_DESERIALIZE_00_H
#define QUANERGY_DESERIALIZE_00_H

#include "deserialize.h"

namespace quanergy
{
  /// Default number of firings per TCP packet
  const int M8_FIRING_PER_PKT = 50;
  /// Ultimately M8 would be a multiecho LiDAR, for now only the first echo is available
  const int M8_NUM_RETURNS = 3;
  /// The total number of lasers on the M8 Sensor
  const int M8_NUM_LASERS = 8;

#pragma pack(push, 1)
  /// \brief structure that holds the sensor firing output
  struct M8FiringData
  {
    std::uint16_t position;
    std::uint16_t padding;
    std::uint32_t returns_distances[M8_NUM_RETURNS][M8_NUM_LASERS];   // 1 cm resolution.
    std::uint8_t  returns_intensities[M8_NUM_RETURNS][M8_NUM_LASERS]; // 255 indicates saturation
    std::uint8_t  returns_status[M8_NUM_LASERS];                      // 0 for now
  }; // 132 bytes

  /// \brief structure that holds multiple sensor firings and gets sent in the TCP packet
  struct DataPacket00
  {
    M8FiringData  data[M8_FIRING_PER_PKT];
    std::uint32_t seconds;     // seconds from Jan 1 1970
    std::uint32_t nanoseconds; // fractional seconds turned to nanoseconds
    std::uint16_t version;     // API version number
    std::uint16_t status;      // 0: good, 1: Sensor SW/FW mismatch
  }; // 6612 bytes
#pragma pack(pop)

  inline void deserializeCopy(const DataPacket00& network_order, DataPacket00& host_order)
  {

  }
}
#endif
