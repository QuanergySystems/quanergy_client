/****************************************************************************
 **
 ** Copyright(C) 2015-- Quanergy Systems. All Rights Reserved.
 ** Contact: http://www.quanergy.com
 **
 ****************************************************************************/

/** \file polar_to_cart.h
 *
 *  \brief Converts point clouds from HVDIR to XYZIR.
 */

#ifndef QUANERGY_POLAR_TO_CART_H
#define QUANERGY_POLAR_TO_CART_H

#include <memory>

#include <boost/signals2.hpp>

#include <pcl/point_cloud.h>

#include <quanergy/point_hvdir.h>
#include <quanergy/client/pointcloud_types.h>


namespace quanergy
{
  namespace client
  {
    struct PolarToCartFilter
    {
      typedef std::shared_ptr<PolarToCartFilter> Ptr;

      typedef boost::signals2::signal<void (PointCloudXYZIRConstPtr const &)> Signal;

      boost::signals2::connection connect(const typename Signal::slot_type& subscriber);

      void slot(PointCloudHVDIRConstPtr const &);

    private:

      Signal signal_;
    };

  } // namespace filters

} // namespace quanergy


#endif
