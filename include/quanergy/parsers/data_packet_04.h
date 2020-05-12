/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/**
 *  @file data_packet_04.h
 *
 *  @brief Data structure for Reduced-Bandwidth M-Series Packets (04)
 */

#ifndef QUANERGY_CLIENT_PARSERS_DATA_PACKET_04_H
#define QUANERGY_CLIENT_PARSERS_DATA_PACKET_04_H

#include <quanergy/client/packet_header.h>

// For various constants.
#include <quanergy/client/m_series_data_packet.h>

#include <quanergy/common/dll_export.h>

namespace quanergy
{

  namespace client
  {

#pragma pack(push, 1)
    struct MSeriesDataPacket04Header
    {
      std::uint16_t status;               // 0: good, 1: Sensor SW/FW mismatch
      std::uint8_t return_id;             // 0, 1, or 2
      std::uint8_t reserved;
    };
    
    struct MSeriesFiringData04
    {
      std::uint16_t     position;
      std::uint16_t     reserved;
      std::uint32_t     radius[M_SERIES_NUM_LASERS];
      std::uint8_t      intensity[M_SERIES_NUM_LASERS];
    };

    struct MSeriesDataPacket04
    {
      MSeriesDataPacket04Header data_header;
      MSeriesFiringData04 firings[M_SERIES_FIRING_PER_PKT];
    };    
  
    struct DLLEXPORT DataPacket04
    {
      PacketHeader packet_header;
      MSeriesDataPacket04 data;
    };
  
#pragma pack(pop)

    inline DLLEXPORT void deserialize(const char* network_buffer, MSeriesDataPacket04& object)
    {
      MSeriesDataPacket04 const & network_order = *reinterpret_cast<MSeriesDataPacket04 const *>(network_buffer);

      object.data_header.status         = deserialize(network_order.data_header.status);
      object.data_header.return_id      = deserialize(network_order.data_header.return_id);
      object.data_header.reserved       = deserialize(network_order.data_header.reserved);

      for (int i = 0; i < M_SERIES_FIRING_PER_PKT; ++i)
      {
        object.firings[i].position      = deserialize(network_order.firings[i].position);
        object.firings[i].reserved       = deserialize(network_order.firings[i].reserved);

        for (int j = 0; j < M_SERIES_NUM_LASERS; ++j)
        {
          object.firings[i].radius[j]           = deserialize(network_order.firings[i].radius[j]);
          object.firings[i].intensity[j]        = deserialize(network_order.firings[i].intensity[j]);
        }                                        

      }
    }

    inline DLLEXPORT void deserialize(const char* network_buffer, DataPacket04& object)
    {
      deserialize(network_buffer, object.packet_header);
      network_buffer += sizeof(PacketHeader);
      deserialize(network_buffer, object.data);
    }

  } // namespace client
} // namespace quanergy

#endif

