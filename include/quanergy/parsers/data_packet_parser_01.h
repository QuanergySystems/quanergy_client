/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/**  \file data_packet_parser_01.h
 *
 *   \brief Provide pointcloud parser functionality for data type 0x01.
 */

#ifndef QUANERGY_PARSERS_DATA_PACKET_PARSER_01_H
#define QUANERGY_PARSERS_DATA_PACKET_PARSER_01_H

#include <limits>

#include <quanergy/parsers/data_packet_parser.h>

#include <quanergy/parsers/data_packet_01.h>

#ifdef _MSC_VER
  #define DLLEXPORT __declspec(dllexport)
#else
  #define DLLEXPORT
#endif

namespace quanergy
{
  namespace client
  {
    struct DLLEXPORT DataPacketParser01 : public DataPacketParser
    {
      DataPacketParser01();

      virtual bool validate(const std::vector<char>& packet);

      virtual bool parse(const std::vector<char>& packet, PointCloudHVDIRPtr& result);
    };

  } // namespace client

} // namespace quanergy
#endif
