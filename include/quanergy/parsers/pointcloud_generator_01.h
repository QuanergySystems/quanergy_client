/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/**  \file pointcloud_generator_01.h
 *
 *   \brief Provide pointcloud generator functionality for data type
 *   0x01.
 */

#ifndef QUANERGY_PARSERS_POINTCLOUD_GENERATOR_01_H
#define QUANERGY_PARSERS_POINTCLOUD_GENERATOR_01_H

#include <limits>

#include <quanergy/client/packet_parser.h>

#include <quanergy/common/pointcloud_types.h>

#include <quanergy/parsers/deserialize_01.h>

#ifdef _MSC_VER
  #define DLLEXPORT __declspec(dllexport)
#else
  #define DLLEXPORT
#endif

namespace quanergy
{
  namespace client
  {

    /** \brief specialization for DataPacket01 */
    template <>
    struct DLLEXPORT PacketParser<PointCloudHVDIRPtr, DataPacket01>
      : PacketParserBase<PointCloudHVDIRPtr>
    {
      PacketParser<PointCloudHVDIRPtr, DataPacket01>(std::string const & frame_id)
        : PacketParserBase<PointCloudHVDIRPtr>(frame_id)
      {
      }

      static bool match(std::uint8_t type)
      {
        return type == 0x01;
      }


      inline void parse(std::uint8_t type, const std::vector<char>& packet)
      {
        if (match(type))
        {
          // don't do the work unless someone is listening
          if (signal_ && signal_->num_slots() == 0)
            return;

          DataPacket01 data_packet;
          deserialize(packet.data(), data_packet);

          PointCloudHVDIRPtr pc = PointCloudHVDIRPtr(new PointCloudHVDIR());

          // pcl pointcloud uses microseconds
          pc->header.stamp = 
            std::uint64_t(data_packet.packet_header.seconds) * 1E6 +
            std::uint64_t(data_packet.packet_header.nanoseconds) * 1E-3;

          pc->header.seq = data_packet.data_header.sequence;

          pc->header.frame_id = frame_id_;

          pc->resize(data_packet.data_header.point_count);

          // intermediate angles and value
          double H = 0;
          double V = 0;
          double cosH = 0;

          for (unsigned int i = 0; i < data_packet.data_header.point_count; ++i)
          {
            DataPoint01 const & point = data_packet.data_points[i];
            PointCloudHVDIR::PointType& pc_point = pc->points[i];

            H = static_cast<double>(point.horizontal_angle) * 1E-4;
            V = static_cast<double>(point.vertical_angle) * 1E-4;
            pc_point.d = static_cast<double>(point.range) * 1E-6;

            // convert to standard HVDIR
            cosH = std::cos(H);
            pc_point.h = std::atan2(std::sin(H), cosH * std::cos(V));
            pc_point.v = std::asin(cosH * std::sin(V));

            pc_point.intensity = point.intensity;

            /// This generator provides no ring.
            pc_point.ring = std::numeric_limits<uint16_t>::max();
          }

          // signal
          (*signal_)(pc);
        }
        else
          throw InvalidDataTypeError();
      }
    };

  } // namespace client

} // namespace quanergy
#endif
