/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/**  \file data_packet_parser_m8.h
 *
 *   \brief Provide pointcloud parser functionality for m8 data.
 */

#ifndef QUANERGY_PARSERS_DATA_PACKET_PARSER_M8_H
#define QUANERGY_PARSERS_DATA_PACKET_PARSER_M8_H

#include <quanergy/parsers/data_packet_parser.h>

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

    const double M8_VERTICAL_ANGLES[] = { 
      -0.318505, 
      -0.2692, 
      -0.218009, 
      -0.165195, 
      -0.111003, 
      -0.0557982, 
      0.f, 
      0.0557982 };

    const std::int32_t M8_NUM_ROT_ANGLES = 10400;

    enum class ReturnSelection { MAX = 0, FIRST, LAST, ALL };

    /** \brief Not a specialization because it is intended to be used by others. */
    struct DLLEXPORT DataPacketParserM8 : public DataPacketParser
    {
      DataPacketParserM8();

      bool parse(const M8DataPacket& data_packet, PointCloudHVDIRPtr& result);

      void setReturnSelection(ReturnSelection return_selection);

    protected:
      static void organizeCloud(PointCloudHVDIRPtr & current_pc);

      /// global packet counter
      uint32_t packet_counter_;

      /// global cloud counter
      uint32_t cloud_counter_;

      /// last accounted for azimuth angle
      double last_azimuth_;

      PointCloudHVDIRPtr current_cloud_;

      /// lookup table for horizontal angle
      std::vector<double> horizontal_angle_lookup_table_;

      /// lookup table for vertical angle
      double vertical_angle_lookup_table_[M8_NUM_LASERS];

      /// return selection
      ReturnSelection return_selection_ = ReturnSelection::FIRST;
    };

  } // namespace client

} // namespace quanergy

#endif
