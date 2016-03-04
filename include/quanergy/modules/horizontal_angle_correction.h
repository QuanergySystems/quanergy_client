/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/** \file horizontal_angle_correction.h
 *
 * \brief Applies correction to horizontal angle from motor offset from vertical
 * access
 */

#ifndef HORIZONAL_ANGLE_CORRECTION_H_
#define HORIZONAL_ANGLE_CORRECTION_H_

#include <boost/signals2.hpp>
#include <pcl/point_cloud.h>

#include <quanergy/common/point_hvdir.h>

#include <quanergy/common/pointcloud_types.h>

#ifdef _MSC_VER
  #define DLLEXPORT __declspec(dllexport)
  #define TYPENAME
#else
  #define DLLEXPORT
  #define TYPENAME typename
#endif

namespace quanergy
{
  namespace client
  {
    struct DLLEXPORT HorizontalAngleCorrection
    {
      typedef std::shared_ptr<HorizontalAngleCorrection> Ptr;

      typedef PointCloudHVDIRPtr ResultType;

      typedef boost::signals2::signal<void (const ResultType&)> Signal;

      // We only want this object created with the correction values
      HorizontalAngleCorrection() = delete;

      /** Constructor */
      HorizontalAngleCorrection(double amplitude, double phase_offset);

      /** 
       * @brief Adds subscriber to be called after this classes functionality
       * 
       * @param subscriber[in] Subscriber to be called.
       * 
       * @return 
       */
      boost::signals2::connection connect(const TYPENAME Signal::slot_type& subscriber);

      /** 
       * @brief Function to call from previous node in chain. Entry point for
       * this classes operation.
       * 
       * @param pc[in] Pointer to point cloud to be processed 
       */
      void slot(PointCloudHVDIRPtr const & pc);

    private:

      /** 
       * @brief Function to apply horizontal angle correction
       * 
       * @param point[inout] Point to have h value corrected. This operation is
       * done in place.
       */
      void correctPoint(PointCloudHVDIR::PointType & point);

      /** signal object to notify next slot */
      Signal signal_;

      /** Amplitude after modeling horizontal angle error. This value is used to
       * apply the correction. */
      double amplitude_ = 0.;

      /** Phase offset after modeling horizontal angle error. */
      double phase_offset_ = 0.;

    };
  
  } /* client */
} /* quanergy */ 

#endif /* end of include guard: HORIZONAL_ANGLE_CORRECTION_H_ */
