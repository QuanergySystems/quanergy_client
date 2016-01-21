/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <quanergy/modules/polar_to_cart_converter.h>

namespace quanergy
{
  namespace client
  {

    boost::signals2::connection PolarToCartConverter::connect(const TYPENAME Signal::slot_type& subscriber)
    {
      return signal_.connect(subscriber);
    }

    void PolarToCartConverter::slot(PointCloudHVDIRConstPtr const & cloudPtr)
    {
      if (!cloudPtr) return;

      // Don't do the work unless someone is listening.
      if (signal_.num_slots() == 0) return;

      PointCloudHVDIR const & cloud = *cloudPtr;

      PointCloudXYZIRPtr resultPtr = PointCloudXYZIRPtr(new PointCloudXYZIR());
      
      PointCloudXYZIR & result = *resultPtr;

      result.header.stamp = cloud.header.stamp;
      result.header.seq = cloud.header.seq;
      result.header.frame_id = cloud.header.frame_id;

      result.reserve(cloud.size());

      bool is_dense = cloud.is_dense;

      for (PointCloudHVDIR::const_iterator i = cloud.points.begin();
           i != cloud.points.end();
           ++i)
      {
        PointCloudXYZIR::PointType pt = polarToCart(*i);

        // use points.push_back instead of cloud.push_back wrapper
        // cloud.push_back wrapper resets width and height
        result.points.push_back(pt);

        // Check if the resulting point cloud is no longer dense
        if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z))
        {
            is_dense = false;
        }
      }

      result.width = cloud.width;
      result.height = cloud.height;
      result.is_dense = is_dense;

      signal_(resultPtr);
    }

    PointCloudXYZIR::PointType PolarToCartConverter::polarToCart(PointCloudHVDIR::PointType const & from)
    {
      PointCloudXYZIR::PointType to;

      to.intensity = from.intensity;
      to.ring = from.ring;

      if (std::isnan (from.d))
      {
        to.x = to.y = to.z = std::numeric_limits<float>::quiet_NaN ();
        return to;
      }

      double const cos_horizontal_angle = std::cos(from.h);
      double const sin_horizontal_angle = std::sin(from.h);

      double const cos_vertical_angle = std::cos(from.v);
      double const sin_vertical_angle = std::sin(from.v);

      // get the distance to the XY plane
      double xy_distance = from.d * cos_vertical_angle;

      to.y = static_cast<float> (xy_distance * sin_horizontal_angle);

      to.x = static_cast<float> (xy_distance * cos_horizontal_angle);

      to.z = static_cast<float> (from.d * sin_vertical_angle);

      return to;
    }

  } // namespace client

} // namespace quanergy
