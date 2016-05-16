/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/**  \file data_packet_01.h
  *
  *  \brief Provide deserialization functionality for data packet type 0x01.
  *
  */


#ifndef QUANERGY_PARSERS_DATA_PACKET_01_H
#define QUANERGY_PARSERS_DATA_PACKET_01_H

#include <iostream>

#include <quanergy/client/packet_header.h>

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
    /** \brief data header 0x01 */
    struct DLLEXPORT DataHeader01
    {
      std::uint32_t sequence;
      std::uint32_t status;
      std::uint32_t point_count;
      std::uint32_t reserved;
    };

    /** \brief data point 0x01 */
    struct DLLEXPORT DataPoint01
    {
      std::int16_t  horizontal_angle; // [-31416, 31416], 1/10,000 radians, +left, -right
      std::int16_t  vertical_angle;   // [-31416, 31416], 1/10,000 radians, +up, -down
      std::uint32_t range;            // micrometers
      std::uint16_t intensity;
      std::uint8_t  status;
      std::uint8_t  reserved;
    };
#pragma pack(pop)

    /** \brief data packet 0x01 */
    struct DLLEXPORT DataPacket01
    {
      PacketHeader              packet_header;
      DataHeader01              data_header;
      //! @todo use custom allocator to guarantee alignment
      std::vector<DataPoint01>  data_points;
    };

    inline DLLEXPORT void deserialize(const char* network_buffer, DataHeader01& object)
    {
      const DataHeader01& network_order = *reinterpret_cast<const DataHeader01*>(network_buffer);

      object.sequence    = deserialize(network_order.sequence);
      object.status      = deserialize(network_order.status);
      object.point_count = deserialize(network_order.point_count);
      object.reserved    = deserialize(network_order.reserved);
    }

    inline DLLEXPORT void deserialize(const char* network_buffer, DataPoint01& object)
    {
      const DataPoint01& network_order = *reinterpret_cast<const DataPoint01*>(network_buffer);

      object.horizontal_angle  = deserialize(network_order.horizontal_angle);
      object.vertical_angle    = deserialize(network_order.vertical_angle);
      object.range             = deserialize(network_order.range);
      object.intensity         = deserialize(network_order.intensity);
      object.status            = deserialize(network_order.status);
      object.reserved          = deserialize(network_order.reserved);
    }

    inline DLLEXPORT void deserialize(const char* network_buffer, DataPacket01& object)
    {
      deserialize(network_buffer, object.packet_header);
      network_buffer += sizeof(PacketHeader);
      deserialize(network_buffer, object.data_header);
      network_buffer += sizeof(DataHeader01);

      if (object.packet_header.size != sizeof(PacketHeader) +
          sizeof(DataHeader01) +
          object.data_header.point_count * sizeof(DataPoint01))
      {
        std::cerr << "Invalid sizes: " << object.data_header.point_count
                  << " points and " << object.packet_header.size << " bytes" << std::endl;
        throw SizeMismatchError();
      }

      object.data_points.resize(object.data_header.point_count);
      std::for_each(object.data_points.begin(), object.data_points.end(),
                    [&network_buffer](DataPoint01& point)
                    {
                      deserialize(network_buffer, point);
                      network_buffer += sizeof(DataPoint01);
                    });
    }

  } // namespace client

} // namespace quanergy

#endif
