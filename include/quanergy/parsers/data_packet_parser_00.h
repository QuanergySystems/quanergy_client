/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/**  \file data_packet_parser_00.h
 *
 *   \brief Provide pointcloud parser functionality for data type 0x00.
 */

#ifndef QUANERGY_PARSERS_DATA_PACKET_PARSER_00_H
#define QUANERGY_PARSERS_DATA_PACKET_PARSER_00_H

#include <quanergy/parsers/packet_parser.h>

#include <quanergy/common/pointcloud_types.h>

#include <quanergy/parsers/data_packet_00.h>
#include <quanergy/parsers/data_packet_parser_m_series.h>

#include <quanergy/common/dll_export.h>

namespace quanergy
{
  namespace client
  {
    struct DLLEXPORT DataPacketParser00 : public DataPacketParserMSeries
    {
      DataPacketParser00() = default;

      virtual bool validate(const std::vector<char>& packet) override;

      virtual bool parse(const std::vector<char>& packet, PointCloudHVDIRPtr& result) override;
    };

  } // namespace client

} // namespace quanergy

#endif
