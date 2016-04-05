/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/**  \file m8_data_packet.h
  *
  *  \brief Provide definition of old m8 data packet
  *
  */


#ifndef QUANERGY_CLIENT_M8_DATA_PACKET_H
#define QUANERGY_CLIENT_M8_DATA_PACKET_H

#ifdef _MSC_VER
  #define DLLEXPORT __declspec(dllexport)
#else
  #define DLLEXPORT
#endif

namespace quanergy
{
  namespace client
  {

    /// Default number of firings per TCP packet
    const int M8_FIRING_PER_PKT = 50;
    /// M8 packet supports multiecho return
    const int M8_NUM_RETURNS = 3;
    /// The total number of lasers on the M8 Sensor
    const int M8_NUM_LASERS = 8;

#pragma pack(push, 1)
    /// \brief structure that holds the sensor firing output
    struct DLLEXPORT M8FiringData
    {
      std::uint16_t position;
      std::uint16_t padding;
      std::uint32_t returns_distances[M8_NUM_RETURNS][M8_NUM_LASERS];   // 10 um resolution.
      std::uint8_t  returns_intensities[M8_NUM_RETURNS][M8_NUM_LASERS]; // 255 indicates saturation
      std::uint8_t  returns_status[M8_NUM_LASERS];                      // 0 for now
    }; // 132 bytes

    /// \brief structure that holds multiple sensor firings and gets sent in the TCP packet
    struct DLLEXPORT M8DataPacket
    {
      M8FiringData  data[M8_FIRING_PER_PKT];
      std::uint32_t seconds;     // seconds from Jan 1 1970
      std::uint32_t nanoseconds; // fractional seconds turned to nanoseconds
      std::uint16_t version;     // API version number.  Version 5 uses distance as units of 10 micrometers, <5 is 10mm
      std::uint16_t status;      // 0: good, 1: Sensor SW/FW mismatch
    }; // 6612 bytes
#pragma pack(pop)

  } // namespace client

} // namespace quanergy
#endif
