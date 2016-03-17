/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <cmath>

#include <quanergy/modules/encoder_correction.h>

namespace quanergy
{
namespace client
{
  EncoderCorrection::EncoderCorrection(double amplitude, double phase_offset)
  {
    amplitude_ = amplitude;
    phase_offset_ = phase_offset;
  }

  boost::signals2::connection EncoderCorrection::connect(const TYPENAME Signal::slot_type& subscriber)
  {
    return signal_.connect(subscriber);
  }

  void EncoderCorrection::slot(PointCloudHVDIRPtr const & cloud_ptr)
  {
    if (!cloud_ptr)
      return;

    // return immediately if there are no slots
    if (signal_.num_slots() == 0)
      return;

    PointCloudHVDIR & cloud = *cloud_ptr;

    for (PointCloudHVDIR::iterator i = cloud.points.begin();
         i != cloud.points.end();
         ++i)
    {
      // corrects in place, saves copying other values
      correctPoint(*i);
    }

    signal_(cloud_ptr);

  }

  void EncoderCorrection::correctPoint(PointCloudHVDIR::PointType & point)
  {
    point.h = point.h - (amplitude_ * sin(point.h - phase_offset_));
  }
  
} /* client */
} /* quanergy */ 
