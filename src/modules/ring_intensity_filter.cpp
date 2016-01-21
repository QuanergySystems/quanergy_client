/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <quanergy/modules/ring_intensity_filter.h>

namespace quanergy
{
  namespace client
  {

    RingIntensityFilter::RingIntensityFilter()
    {
      for (uint16_t i = 0; i < M8_NUM_LASERS; ++i)
      {
        ring_filter_range_[i] = 1.0f;
        ring_filter_intensity_[i] = 0.0f;
      }
    }

	boost::signals2::connection RingIntensityFilter::connect(const TYPENAME Signal::slot_type& subscriber)
    {
      return signal_.connect(subscriber);
    }


    void RingIntensityFilter::slot(PointCloudHVDIRConstPtr const & cloudPtr)
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
        PointCloudHVDIR::PointType pt = filterGhosts(*i);

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


    PointCloudHVDIR::PointType RingIntensityFilter::filterGhosts(PointCloudHVDIR::PointType const & from) const
    {
      PointCloudHVDIR::PointType to;

      to.intensity = from.intensity;
      to.ring = from.ring;

      to.h = from.h;
      to.v = from.v;

      to.d = ((from.ring < M8_NUM_LASERS) &&
              (from.d < ring_filter_range_[from.ring]) &&
              (from.intensity < ring_filter_intensity_[from.ring]))
        ?
        std::numeric_limits<float>::quiet_NaN()
        :
        from.d;

      return to;
    }


    float RingIntensityFilter::getRingFilterMinimumRangeThreshold (const std::uint16_t laser_beam) const
    {
      if (laser_beam >= M8_NUM_LASERS)
      {
        std::cerr << "Index out of bound! Beam index should be between 0 and " << M8_NUM_LASERS << std::endl;
        return std::numeric_limits<float>::quiet_NaN();
      }

      return (ring_filter_range_[laser_beam]);
    }


    void RingIntensityFilter::setRingFilterMinimumRangeThreshold (const std::uint16_t laser_beam, 
                                                                  const float threshold)
    {
      if (laser_beam >= M8_NUM_LASERS)
      {
        std::cerr << "Index out of bound! Beam index should be between 0 and " 
                  << M8_NUM_LASERS << std::endl;
      }
      else
      {
        ring_filter_range_[laser_beam] = threshold;
      }
    }


    std::uint8_t RingIntensityFilter::getRingFilterMinimumIntensityThreshold (const uint16_t laser_beam) const
    {
      if (laser_beam >= M8_NUM_LASERS)
      {
        std::cerr << "Index out of bound! Beam index should be between 0 and " 
                  << M8_NUM_LASERS << std::endl;
        return -1;
      }

      return (ring_filter_intensity_[laser_beam]);
    }


    void RingIntensityFilter::setRingFilterMinimumIntensityThreshold (const uint16_t laser_beam, 
                                                                      const std::uint8_t threshold)
    {
      if (laser_beam >= M8_NUM_LASERS)
      {
        std::cerr << "Index out of bound! Beam index should be between 0 and " << M8_NUM_LASERS << std::endl;
      }
      else
      {
        ring_filter_intensity_[laser_beam] = threshold;
      }
    }




  } // namespace client

} // namespace quanergy
