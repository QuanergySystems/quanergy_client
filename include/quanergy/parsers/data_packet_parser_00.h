/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
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
#include <quanergy/parsers/data_packet_parser_m8.h>

#ifdef _MSC_VER
  #define DLLEXPORT __declspec(dllexport)
#else
  #define DLLEXPORT
#endif

namespace quanergy
{
  namespace client
  {
    struct DLLEXPORT DataPacketParser00 : public DataPacketParserM8
    {
      DataPacketParser00();

      virtual bool validate(const std::vector<char>& packet);

      virtual bool parse(const std::vector<char>& packet, PointCloudHVDIRPtr& result);
    };

  } // namespace client

} // namespace quanergy

#endif
