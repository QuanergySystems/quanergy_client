/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <quanergy/modules/horizontal_angle_correction.h>

namespace quanergy
{
namespace client
{
  boost::signals2::connection HorizontalAngleCorrection::connect(const TYPENAME Signal::slot_type& subscriber)
  {
    return signal_.connect(subscriber);
  }

  void HorizontalAngleCorrection::slot(PointCloudHVDIRPtr const & cloud_ptr)
  {
    if (!cloud_ptr)
      return;

    // return immediately if there are no slots
    if (signal_.num_slots() == 0)
      return;

    PointCloudHVDIR & cloud = *cloud_ptr;

    //PointCloudXYZIRPtr result_ptr = PointCloudXYZIRPtr(new PointCloudXYZIR());
    //PointCloudXYZIR & result = *result_ptr;

    // result.header.stamp = cloud.header.stamp;
    // result.header.seq = cloud.header.seq;
    // result.header.frame_id = cloud.header.frame_id;

    // result.reserve(cloud.size());

    for (PointCloudHVDIR::iterator i = cloud.points.begin();
         i != cloud.points.end();
         ++i)
    {
      // corrects in place, saves copying other values
      correctPoint(*i);
    }

    signal_(cloud_ptr);

  }

  void HorizontalAngleCorrection::correctPoint(PointCloudHVDIR::PointType & point)
  {
    point.h++;
  }
  
} /* client */
} /* quanergy */ 
