/****************************************************************
 **                                                            **
 **  Copyright(C) 2016 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/** \file data_packet_parser.h
 * \brief specialize packet parser for HVDIR data out
 */

#ifndef QUANERGY_CLIENT_DATA_PACKET_PARSER_H
#define QUANERGY_CLIENT_DATA_PACKET_PARSER_H

#include <quanergy/parsers/packet_parser.h>

#include <quanergy/common/pointcloud_types.h>

namespace quanergy
{
  namespace client
  {
    /** \brief base class for data packet parsers */
    struct DataPacketParser : public PacketParserBase<PointCloudHVDIRPtr>
    {
      DataPacketParser() = default;

      /// common interface for setting frame_id which should be put into resulting pointcloud
      void setFrameId(const std::string frame_id)
      {
        frame_id_ = frame_id;
      }

    protected:
      std::string frame_id_;
    };
  } // namespace client

} // namespace quanergy
#endif
