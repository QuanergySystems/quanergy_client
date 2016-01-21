/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <quanergy/modules/distance_filter.h>

#include <limits>

namespace quanergy
{
  namespace client
  {

    DistanceFilter::DistanceFilter()
      : max_distance_threshold_(std::numeric_limits<float>::max())
      , min_distance_threshold_(0.0f) {}


    boost::signals2::connection DistanceFilter::connect(const TYPENAME Signal::slot_type& subscriber)
    {
      return signal_.connect(subscriber);
    }


    void DistanceFilter::slot(PointCloudHVDIRConstPtr const & cloudPtr)
    {
      if (!cloudPtr) return;

      // Don't do the work unless someone is listening.
      if (signal_.num_slots() == 0) return;

      PointCloudHVDIR const & cloud = *cloudPtr;

      PointCloudHVDIRPtr resultPtr = PointCloudHVDIRPtr(new PointCloudHVDIR());
      
      PointCloudHVDIR & result = *resultPtr;

      result.header.stamp = cloud.header.stamp;
      result.header.seq = cloud.header.seq;
      result.header.frame_id = cloud.header.frame_id;

      result.reserve(cloud.size());

      bool is_dense = cloud.is_dense;

      for (PointCloudHVDIR::const_iterator i = cloud.points.begin();
           i != cloud.points.end();
           ++i)
      {
        PointCloudHVDIR::PointType pt = filterByDistance(*i);

        result.points.push_back(pt);

        // Check if the resulting point cloud is no longer dense
        if (std::isnan(pt.d))
        {
            is_dense = false;
        }
      }

      result.width = cloud.width;
      result.height = cloud.height;
      result.is_dense = is_dense;

      signal_(resultPtr);
    }

    PointCloudHVDIR::PointType DistanceFilter::filterByDistance(PointCloudHVDIR::PointType const & from)
    {
      PointCloudHVDIR::PointType to;

      to.intensity = from.intensity;
      to.ring = from.ring;

      to.h = from.h;
      to.v = from.v;

      to.d = ((from.d < min_distance_threshold_) ||
              (from.d > max_distance_threshold_))
        ?
        std::numeric_limits<float>::quiet_NaN()
        :
        from.d;

      return to;
    }


    void DistanceFilter::setMaximumDistanceThreshold(float maxThreshold) {
      max_distance_threshold_ = maxThreshold;
    }


    float DistanceFilter::getMaximumDistanceThreshold() const {
      return max_distance_threshold_;
    }


    void DistanceFilter::setMinimumDistanceThreshold(float minThreshold) {
      min_distance_threshold_ = minThreshold;
    }


    float DistanceFilter::getMinimumDistanceThreshold() const {
      return min_distance_threshold_;
    }

  } // namespace client

} // namespace quanergy
