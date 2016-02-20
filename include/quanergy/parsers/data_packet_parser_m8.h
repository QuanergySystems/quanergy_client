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

#include <quanergy/parsers/packet_parser.h>

#include <quanergy/common/pointcloud_types.h>

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

    /** \brief Not a specialization because it is intended to be used by others. */
    struct DLLEXPORT DataPacketParserM8 : PacketParserBase<PointCloudHVDIRPtr>
    {
      DataPacketParserM8()
        : PacketParserBase<PointCloudHVDIRPtr>()
        , packet_counter_(0)
        , cloud_counter_(0)
        , last_azimuth_(65000)
        , current_cloud_(new PointCloudHVDIR())
        , horizontal_angle_lookup_table_(M8_NUM_ROT_ANGLES+1)
      {
        for (std::uint32_t i = 0; i <= M8_NUM_ROT_ANGLES; i++)
        {
          // Shift by half the rot angles to keep the number positive when wrapping.
          std::uint32_t j = (i + M8_NUM_ROT_ANGLES/2) % M8_NUM_ROT_ANGLES;

          // normalized
          double n = static_cast<double>(j) / static_cast<double>(M8_NUM_ROT_ANGLES);

          double rad = n * M_PI * 2.0 - M_PI;

          horizontal_angle_lookup_table_[i] = rad;
        }

        const double* angle_in_radians = M8_VERTICAL_ANGLES;
        for (std::uint32_t i = 0; i < M8_NUM_LASERS; ++i, ++angle_in_radians)
        {
          vertical_angle_lookup_table_[i] = *angle_in_radians;
        }
      }

      virtual inline bool parse(const M8DataPacket& data_packet, PointCloudHVDIRPtr& result)
      {
        bool ret = false;
        if (data_packet.status != 0)
        {
          std::cerr << "Sensor status nonzero: " << data_packet.status;
          if (data_packet.status == 1)
            throw FirmwareVersionMismatchError();
          return ret; // don't process if sensor in error
        }

        // this time is used for the cloud stamp which is a 64 bit integer in units of microseconds
        std::uint64_t time;
        if (data_packet.version <= 3)
        {
          // early versions of API put 10 ns increments in this field
          time = data_packet.seconds * 1E6 + data_packet.nanoseconds * 1E-2;
        }
        else
          time = data_packet.seconds * 1E6 + data_packet.nanoseconds * 1E-3;

        ++packet_counter_;

        // get spin direction
        int direction = 0;
        if (data_packet.data[0].position - data_packet.data[M8_FIRING_PER_PKT-1].position > 0)
          direction = (data_packet.data[0].position - data_packet.data[M8_FIRING_PER_PKT-1].position > 4000) ? 1 : -1;
        else
          direction = (data_packet.data[M8_FIRING_PER_PKT-1].position - data_packet.data[0].position > 4000) ? -1 : 1;

        // for each firing
        for (int i = 0; i < M8_FIRING_PER_PKT; ++i)
        {
          const M8FiringData &data = data_packet.data[i];

          // calculate the angle in degrees
          double azimuth_angle = (static_cast<double> ((data.position+(M8_NUM_ROT_ANGLES/2))%M8_NUM_ROT_ANGLES) / (M8_NUM_ROT_ANGLES) * 360.0) - 180.;
          // check for wrap which indicates completion of a scan
          if (direction * azimuth_angle < direction * last_azimuth_)
          {
            if (current_cloud_->size () > 0)
            {
              organizeCloud(current_cloud_);

              current_cloud_->header.stamp = time;
              current_cloud_->header.seq = cloud_counter_;

              ++cloud_counter_;

              // fire the signal that we have a new cloud
              result = current_cloud_;
              ret = true;
            }

            // start a new cloud
            current_cloud_.reset(new PointCloudHVDIR());
            // at first we assume it is dense
            current_cloud_->is_dense = true;
          }

          double const horizontal_angle = horizontal_angle_lookup_table_[data.position];

          for (int j = 0; j < M8_NUM_LASERS; j++)
          {
            // convert range to meters
            float range = data.returns_distances[0][j] * .01;
            unsigned char intensity = data.returns_intensities[0][j];

            if (range < 1E-4)
              range = std::numeric_limits<float>::quiet_NaN ();

            // output point
            PointCloudHVDIR::PointType hvdir;

            double const vertical_angle = vertical_angle_lookup_table_[j];

            hvdir.h = horizontal_angle;
            hvdir.v = vertical_angle;
            hvdir.d = range;

            hvdir.intensity = intensity;
            hvdir.ring = j;

            // add the point to the current scan
            current_cloud_->push_back (hvdir);

            // if the range is NaN, the cloud is not dense, one point is sufficient
            if (current_cloud_->is_dense && std::isnan (range))
              current_cloud_->is_dense = false;
          }

          last_azimuth_ = azimuth_angle;
        }

        return ret;
      }

    protected:
      static void organizeCloud(PointCloudHVDIRPtr & current_pc)
      {
        // transpose the cloud
        PointCloudHVDIRPtr temp_pc(new PointCloudHVDIR());  

        temp_pc->header.stamp = current_pc->header.stamp;
        temp_pc->header.seq = current_pc->header.seq;
        temp_pc->header.frame_id = current_pc->header.frame_id;

        // reserve space
        temp_pc->reserve(current_pc->size());

        unsigned int temp_index;
        unsigned int width = current_pc->size () / M8_NUM_LASERS; // CONSTANT FOR NUM BEAMS

        // iterate through each ring from top down
        for (int i = M8_NUM_LASERS - 1; i >= 0; --i)
        {
          // iterate through width in collect order
          for (unsigned int j = 0; j < width; ++j)
          {
            // original data is in collect order and laser order
            temp_index = j * M8_NUM_LASERS + i;

            temp_pc->push_back(current_pc->points[temp_index]);
          }
        }

        current_pc.swap(temp_pc);

        current_pc->height = M8_NUM_LASERS;
        current_pc->width  = width;
      }

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
    };

  } // namespace client

} // namespace quanergy

#endif
