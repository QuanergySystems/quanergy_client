/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/** \file ring_intensity_filter.h
 *
 *  \brief Filters HVDIR points based on intensity for a given ring.
 *
 *  This filter is designed to remove ring artifacts that are sometimes found in prototype M-Series sensors
 */

#ifndef QUANERGY_MODULES_RING_INTENSITY_FILTER_H
#define QUANERGY_MODULES_RING_INTENSITY_FILTER_H

#include <memory>
#include <cstdint>

#include <boost/signals2.hpp>

#include <pcl/point_cloud.h>

#include <quanergy/common/point_hvdir.h>
#include <quanergy/common/pointcloud_types.h>

// For M_SERIES_NUM_LASERS
#include <quanergy/client/m_series_data_packet.h>

#include <quanergy/common/dll_export.h>

namespace quanergy
{
  namespace client
  {
    struct DLLEXPORT RingIntensityFilter
    {
      typedef std::shared_ptr<RingIntensityFilter> Ptr;

      typedef PointCloudHVDIRPtr ResultType;

      typedef boost::signals2::signal<void (const ResultType&)> Signal;

      RingIntensityFilter();

      boost::signals2::connection connect(const typename Signal::slot_type& subscriber);

      void slot(PointCloudHVDIRConstPtr const &);

      /** \brief For ring filtering: Returns the minimum range filter threshold for the given beam, in meters */
      float getRingFilterMinimumRangeThreshold (const std::uint16_t laser_beam) const;

      /** \brief For ring filtering: Set the minimum range filter threshold for the given beam, in meters
        * This value is in meters. Defaults to 1.0
        */
      void setRingFilterMinimumRangeThreshold (const std::uint16_t laser_beam, const float min_threshold);

      /** \brief For ring filtering: Returns the minimum intensity filter threshold for the given beam */
      uint8_t getRingFilterMinimumIntensityThreshold (const std::uint16_t laser_beam) const;

      /** \brief For ring filtering: Set the minimum intensity filter threshold for the given beam, in meters
        * This value is an integer between 0-255. Defaults to 0
        */
      void setRingFilterMinimumIntensityThreshold (const uint16_t laser_beam, const uint8_t min_threshold);

    private:

      PointCloudHVDIR::PointType filterGhosts(PointCloudHVDIR::PointType const & from) const;

      Signal signal_;

      float ring_filter_range_[M_SERIES_NUM_LASERS];
      std::uint8_t ring_filter_intensity_[M_SERIES_NUM_LASERS];
    };

  } // namespace filters

} // namespace quanergy


#endif
