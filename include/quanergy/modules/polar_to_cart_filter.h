/****************************************************************************
 **
 ** Copyright(C) 2015-- Quanergy Systems. All Rights Reserved.
 ** Contact: http://www.quanergy.com
 **
 ****************************************************************************/

/** \file polar_to_cart_filter.h
 *
 *  \brief Converts point clouds from polar coordinates to cartesian
 *  coordinates, i.e. HVDIR to XYZIR.
 */

#ifndef QUANERGY_POLAR_TO_CART_FILTER_H
#define QUANERGY_POLAR_TO_CART_FILTER_H

#include <memory>

#include <boost/signals2.hpp>

#include <pcl/point_cloud.h>

#include <quanergy/point_hvdir.h>

#include <quanergy/common/pointcloud_types.h>


namespace quanergy
{
  namespace client
  {
    struct PolarToCartFilter
    {
      typedef std::shared_ptr<PolarToCartFilter> Ptr;

      typedef PointCloudXYZIRPtr Result;

      typedef boost::signals2::signal<void (Result const &)> Signal;

      boost::signals2::connection connect(const typename Signal::slot_type& subscriber);

      void slot(PointCloudHVDIRConstPtr const &);

    private:

      static PointCloudXYZIR::PointType polarToCart(PointCloudHVDIR::PointType const & from);

      Signal signal_;
    };

  } // namespace filters

} // namespace quanergy


#endif
