/****************************************************************
 **                                                            **
 **  Copyright(C) 2017 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/**
 *  @file data_packet_parser_04.h
 *
 *  @brief Provide pointcloud parser functionality for data type 0x04.
 */

#ifndef QUANERGY_CLIENT_PARSERS_DATA_PACKET_PARSER_04_H
#define QUANERGY_CLIENT_PARSERS_DATA_PACKET_PARSER_04_H

#include <quanergy/parsers/packet_parser.h>

#include <quanergy/parsers/data_packet_04.h>

#include <quanergy/parsers/data_packet_parser_m_series.h>

#include <quanergy/common/dll_export.h>

namespace quanergy
{
  namespace client
  {
    struct DLLEXPORT DataPacketParser04 : DataPacketParserMSeries
    {
      // Constructor
      DataPacketParser04() : DataPacketParserMSeries()
      {
        // For 04 packets, return selection is done on the sensor, so if not explicitly set,
        // we will process whatever 04 packets we receive.
        return_selection_ = quanergy::client::ALL_RETURNS;
      }

      virtual bool validate(const std::vector<char>& packet) override;
  
      bool parse(DataPacket04 const & data_packet, PointCloudHVDIRPtr& result);

      virtual bool parse(const std::vector<char>& packet, PointCloudHVDIRPtr& result) override;

    };

  } // namespace client

} // namespace quanergy

#endif
