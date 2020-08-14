/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/** \file distance_filter.h
 *
 *  \brief Filters HVDIR points based on radius distance (by setting
 *  them to NAN).
 */

#ifndef QUANERGY_MODULES_DISTANCE_FILTER_H
#define QUANERGY_MODULES_DISTANCE_FILTER_H

#include <memory>

#include <boost/signals2.hpp>

#include <pcl/point_cloud.h>

#include <quanergy/common/point_hvdir.h>
#include <quanergy/common/pointcloud_types.h>

#include <quanergy/common/dll_export.h>

namespace quanergy
{
  namespace client
  {
    struct DLLEXPORT DistanceFilter
    {
      typedef std::shared_ptr<DistanceFilter> Ptr;

      typedef PointCloudHVDIRPtr ResultType;

      typedef boost::signals2::signal<void (const ResultType&)> Signal;

      DistanceFilter();

      boost::signals2::connection connect(const typename Signal::slot_type& subscriber);

      void slot(PointCloudHVDIRConstPtr const &);

      void setMaximumDistanceThreshold(float maxThreshold);
      float getMaximumDistanceThreshold() const;

      void setMinimumDistanceThreshold(float minThreshold);
      float getMinimumDistanceThreshold() const;

    private:

      PointCloudHVDIR::PointType filterByDistance(PointCloudHVDIR::PointType const & from);

      Signal signal_;

      float max_distance_threshold_;
      float min_distance_threshold_;
    };

  } // namespace filters

} // namespace quanergy


#endif
