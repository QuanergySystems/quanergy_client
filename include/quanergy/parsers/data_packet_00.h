/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/**  \file data_packet_00.h
  *
  *  \brief Provide deserialization functionality for data packet type 0x00
  *
  *  data packet 00 is a wrapper for the old m8 data with the new header.
  *
  */


#ifndef QUANERGY_PARSERS_DATA_PACKET_00_H
#define QUANERGY_PARSERS_DATA_PACKET_00_H

#include <quanergy/client/packet_header.h>
#include <quanergy/client/m8_data_packet.h>

#ifdef _MSC_VER
  #define DLLEXPORT __declspec(dllexport)
#else
  #define DLLEXPORT
#endif

namespace quanergy
{
  namespace client
  {

#pragma pack(push, 1)
    /** \brief data packet 0x00 */
    struct DLLEXPORT DataPacket00
    {
      PacketHeader packet_header;
      M8DataPacket data_body;
    };
#pragma pack(pop)

    inline DLLEXPORT void deserialize(const char* network_buffer, M8FiringData& object)
    {
      const M8FiringData& network_order = *reinterpret_cast<const M8FiringData*>(network_buffer);

      object.position = deserialize(network_order.position);
      object.padding  = deserialize(network_order.padding);

      // deserialize each range
      const std::uint32_t* net_d_ptr = reinterpret_cast<const std::uint32_t*>(network_order.returns_distances);
      std::uint32_t* obj_d_ptr = reinterpret_cast<std::uint32_t*>(object.returns_distances);
      std::for_each(obj_d_ptr, obj_d_ptr + M8_NUM_RETURNS * M8_NUM_LASERS,
                    [&net_d_ptr](std::uint32_t& range)
                    {
                      range = deserialize(*net_d_ptr);
                      ++net_d_ptr;
                    });

      // deserialize each intensity
      const std::uint8_t* net_i_ptr = reinterpret_cast<const std::uint8_t*>(network_order.returns_intensities);
      std::uint8_t* obj_i_ptr = reinterpret_cast<std::uint8_t*>(object.returns_intensities);
      std::for_each(obj_i_ptr, obj_i_ptr + M8_NUM_RETURNS * M8_NUM_LASERS,
                    [&net_i_ptr](std::uint8_t& intensity)
                    {
                      intensity = deserialize(*net_i_ptr);
                      ++net_i_ptr;
                    });

      // deserialize each status
      const std::uint8_t* net_s_ptr = reinterpret_cast<const std::uint8_t*>(network_order.returns_status);
      std::uint8_t* obj_s_ptr = reinterpret_cast<std::uint8_t*>(object.returns_status);
      std::for_each(obj_s_ptr, obj_s_ptr + M8_NUM_LASERS,
                    [&net_s_ptr](std::uint8_t& status)
                    {
                      status = deserialize(*net_s_ptr);
                      ++net_s_ptr;
                    });
    }

    inline DLLEXPORT void deserialize(const char* network_buffer, M8DataPacket& object)
    {
      const M8DataPacket& network_order = *reinterpret_cast<const M8DataPacket*>(network_buffer);

      object.seconds      = deserialize(network_order.seconds);
      object.nanoseconds  = deserialize(network_order.nanoseconds);
      object.version      = deserialize(network_order.version);
      object.status       = deserialize(network_order.status);

      // deserialize each firing
      M8FiringData* obj_d_ptr = reinterpret_cast<M8FiringData*>(object.data);
      std::for_each(obj_d_ptr, obj_d_ptr + M8_FIRING_PER_PKT,
                    [&network_buffer](M8FiringData& firing)
                    {
                      deserialize(network_buffer, firing);
                      network_buffer += sizeof(M8FiringData);
                    });
    }

    inline DLLEXPORT void deserialize(const char* network_buffer, DataPacket00& object)
    {
      deserialize(network_buffer, object.packet_header);
      network_buffer += sizeof(PacketHeader);
      deserialize(network_buffer, object.data_body);
    }

  } // namespace client

} // namespace quanergy
#endif
