#include <quanergy/client/polar_to_cart_filter.h>

namespace quanergy
{
  namespace client
  {

    boost::signals2::connection PolarToCartFilter::connect(const typename Signal::slot_type& subscriber)
    {
      return signal_.connect(subscriber);
    }

    void PolarToCartFilter::slot(PointCloudHVDIRConstPtr const & cloudPtr)
    {
      if (!cloudPtr) return;

      PointCloudHVDIR const & cloud = *cloudPtr;

      PointCloudXYZIRPtr resultPtr = PointCloudXYZIRPtr(new PointCloudXYZIR());
      
      PointCloudXYZIR & result = *resultPtr;

      result.header.stamp = cloud.header.stamp;
      result.header.seq = cloud.header.seq;
    }

  } // namespace client

} // namespace quanergy
