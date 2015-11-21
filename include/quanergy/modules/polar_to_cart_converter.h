/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/** \file polar_to_cart_converter.h
 *
 *  \brief Converts point clouds from polar coordinates to cartesian
 *  coordinates, i.e. HVDIR to XYZIR.
 */

#ifndef QUANERGY_MODULES_POLAR_TO_CART_CONVERTER_H
#define QUANERGY_MODULES_POLAR_TO_CART_CONVERTER_H

#include <memory>

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
    struct DLLEXPORT PolarToCartConverter
    {
      typedef std::shared_ptr<PolarToCartConverter> Ptr;

      typedef PointCloudXYZIRPtr Result;

      typedef boost::signals2::signal<void (Result const &)> Signal;

	  boost::signals2::connection connect(const TYPENAME Signal::slot_type& subscriber);

      void slot(PointCloudHVDIRConstPtr const &);

    private:

      static PointCloudXYZIR::PointType polarToCart(PointCloudHVDIR::PointType const & from);

      Signal signal_;
    };

  } // namespace client

} // namespace quanergy


#endif
