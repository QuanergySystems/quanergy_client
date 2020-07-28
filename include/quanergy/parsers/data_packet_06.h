/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/**
 *  @file data_packet_06.h
 *
 *  @brief Data structure for M1 Packets (06)
 */

#ifndef QUANERGY_CLIENT_PARSERS_DATA_PACKET_06_H
#define QUANERGY_CLIENT_PARSERS_DATA_PACKET_06_H

#include <quanergy/client/packet_header.h>

// For various constants.
#include <quanergy/client/m_series_data_packet.h>

#include <quanergy/common/dll_export.h>

#include <boost/static_assert.hpp>

namespace quanergy
{

  namespace client
  {

#pragma pack(push, 1)
    /// \brief structure that holds the sensor firing
    template<std::uint8_t NUM_RETURNS>
    struct M1FiringData
    {
      BOOST_STATIC_ASSERT(NUM_RETURNS == 1 || NUM_RETURNS == M_SERIES_NUM_RETURNS);

      std::uint16_t position = 0;
      std::uint16_t reserved = 0;
      std::uint32_t radius[NUM_RETURNS];   // 10 um resolution.
      std::uint8_t  intensity[NUM_RETURNS]; // 255 indicates saturation
      std::uint8_t  padding[4 - NUM_RETURNS];
    };

    /// \brief structure to hold data header info
    struct M1DataHeader
    {
      std::uint16_t status;    // 0: good, 1: Sensor SW/FW mismatch
      std::uint8_t return_id;  // 0, 1, 2, or 3 (all)
      std::uint8_t reserved;
    };

    /// \brief structure that holds multiple sensor firings
    template<std::uint8_t NUM_RETURNS>
    struct M1DataPacket
    {
      M1FiringData<NUM_RETURNS> firings[M_SERIES_FIRING_PER_PKT];
    };

    /// \brief M1 packet
    template<std::uint8_t NUM_RETURNS>
    struct DLLEXPORT DataPacket06
    {
      PacketHeader               packet_header;
      M1DataHeader               data_header;
      M1DataPacket<NUM_RETURNS>  data;
    };
  
#pragma pack(pop)

    inline DLLEXPORT void deserialize(const char* network_buffer, M1DataHeader& header)
    {
      M1DataHeader const & network_order = *reinterpret_cast<M1DataHeader const *>(network_buffer);

      header.status     = deserialize(network_order.status);
      header.return_id  = deserialize(network_order.return_id);
      header.reserved   = deserialize(network_order.reserved);
    }

    /// template function to deserialize only allowing 1 or 3 returns
    template<std::uint8_t R>
    inline DLLEXPORT typename std::enable_if<R == 1 || R == 3, void>::type deserialize(
        const char* network_buffer, M1DataPacket<R>& object)
    {
      M1DataPacket<R> const & network_order = *reinterpret_cast<M1DataPacket<R> const *>(network_buffer);

      for (int i = 0; i < M_SERIES_FIRING_PER_PKT; ++i)
      {
        object.firings[i].position = deserialize(network_order.firings[i].position);
        object.firings[i].reserved = deserialize(network_order.firings[i].reserved);

        for (int j = 0; j < R; ++j)
        {
          object.firings[i].radius[j]     = deserialize(network_order.firings[i].radius[j]);
          object.firings[i].intensity[j]  = deserialize(network_order.firings[i].intensity[j]);
          std::memcpy(object.firings[i].padding, network_order.firings[i].padding, sizeof(M1FiringData<R>::padding));
        }                                        
      }
    }

    template<std::uint8_t R>
    inline DLLEXPORT typename std::enable_if<R == 1 || R == 3, void>::type deserialize(
        const char* network_buffer, DataPacket06<R>& object)
    {
      deserialize(network_buffer, object.packet_header);
      network_buffer += sizeof(PacketHeader);
      deserialize(network_buffer, object.data_header);
      network_buffer += sizeof(M1DataHeader);
      deserialize(network_buffer, object.data);
    }

    #pragma pack(push, 1)
    


#pragma pack(pop)


  } // namespace client
} // namespace quanergy

#endif

