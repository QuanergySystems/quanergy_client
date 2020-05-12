/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/**  \file m_series_data_packet.h
  *
  *  \brief Provide definition of the M-Series data packet body
  *
  */


#ifndef QUANERGY_CLIENT_M_SERIES_DATA_PACKET_H
#define QUANERGY_CLIENT_M_SERIES_DATA_PACKET_H

#include <cstdint>

#include <quanergy/common/dll_export.h>

namespace quanergy
{
  namespace client
  {

    /// Default number of firings per TCP packet
    const int M_SERIES_FIRING_PER_PKT = 50;
    /// M-Series packet supports multiecho return
    const int M_SERIES_NUM_RETURNS = 3;
    /// The total number of lasers on the M-Series Sensors
    const int M_SERIES_NUM_LASERS = 8;

    /**
     *  \brief StatusType is a 16-bit bitfield that defines the know status flags
     *         possible in the MSeriesDataPacket status field.
     */
    enum struct StatusType : std::uint16_t
    {
      GOOD                   = 0, ///< Good data
      SENSOR_SW_FW_MISMATCH  = 1 << 0, ///< Sensor SW/FW mismatch
      WATCHDOG_VIOLATION     = 1 << 1 ///< Internal software not responding
    };

#pragma pack(push, 1)
    /// \brief structure that holds the sensor firing output
    struct DLLEXPORT MSeriesFiringData
    {
      std::uint16_t position;
      std::uint16_t padding;
      std::uint32_t returns_distances[M_SERIES_NUM_RETURNS][M_SERIES_NUM_LASERS];   // 10 um resolution.
      std::uint8_t  returns_intensities[M_SERIES_NUM_RETURNS][M_SERIES_NUM_LASERS]; // 255 indicates saturation
      std::uint8_t  returns_status[M_SERIES_NUM_LASERS];                      // 0 for now
    }; // 132 bytes

    /// \brief structure that holds multiple sensor firings and gets sent in the TCP packet
    struct DLLEXPORT MSeriesDataPacket
    {
      MSeriesFiringData  data[M_SERIES_FIRING_PER_PKT];
      std::uint32_t seconds;     // seconds from Jan 1 1970
      std::uint32_t nanoseconds; // fractional seconds turned to nanoseconds
      std::uint16_t version;     // API version number.  Version 5 uses distance as units of 10 micrometers, <5 is 10mm
      std::uint16_t status;      // 0: good, 1: Sensor SW/FW mismatch
    }; // 6612 bytes
#pragma pack(pop)

  } // namespace client

} // namespace quanergy
#endif
