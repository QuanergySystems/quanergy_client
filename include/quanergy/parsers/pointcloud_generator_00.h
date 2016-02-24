/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/**  \file pointcloud_generator_00.h
 *
 *   \brief Provide pointcloud generator functionality for data type 0x00.
 */

#ifndef QUANERGY_PARSERS_POINTCLOUD_GENERATOR_00_H
#define QUANERGY_PARSERS_POINTCLOUD_GENERATOR_00_H

#include <quanergy/client/packet_parser.h>

#include <quanergy/common/pointcloud_types.h>

#include <quanergy/parsers/deserialize_00.h>
#include <quanergy/parsers/pointcloud_generator_m8.h>

#ifdef _MSC_VER
  #define DLLEXPORT __declspec(dllexport)
#else
  #define DLLEXPORT
#endif

namespace quanergy
{
  namespace client
  {

    /** \brief specialization for DataPacket00 */
    template <>
    struct DLLEXPORT PacketParser<PointCloudHVDIRPtr, DataPacket00>
      : public PointCloudGeneratorM8
    {
    public:
      PacketParser<PointCloudHVDIRPtr, DataPacket00>(std::string const & frame_id)
        : PointCloudGeneratorM8(frame_id)
      {
      }

      static bool match(std::uint8_t type)
      {
        return type == 0x00;
      }

      inline void parse(std::uint8_t type, const std::vector<char>& packet)
      {
        if (match(type))
        {
          DataPacket00 data_packet;
          deserialize(packet.data(), data_packet);
          PointCloudGeneratorM8::parse(data_packet.data_body);
        }
        else
          throw InvalidDataTypeError();
      }
    };

  } // namespace client

} // namespace quanergy

#endif
