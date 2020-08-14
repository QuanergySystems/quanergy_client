/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
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
      DataPacketParser04() = default;

      virtual bool validate(const std::vector<char>& packet) override;
  
      virtual bool parse(const std::vector<char>& packet, PointCloudHVDIRPtr& result) override;

    };

  } // namespace client

} // namespace quanergy

#endif
