/****************************************************************************
 **
 ** Copyright(C) 2015-- Quanergy Systems. All Rights Reserved.
 ** Contact: http://www.quanergy.com
 **
 ****************************************************************************/

/** \file pointcloud_types.h
 *
 *  \brief Provide typedefs for standard point clouds.
 */

#ifndef QUANERGY_POINTCLOUD_TYPES_H
#define QUANERGY_POINTCLOUD_TYPES_H

#include <memory>
#include <boost/shared_ptr.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace quanergy {

  /*!
   *  Note the PCL uses boost shared pointers internally.
   *  @TODO: Switch to std::shared_ptr?
   */

  typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;

#if 0
  typedef std::shared_ptr<PointCloudXYZI> PointCloudXYZIPtr;
#else
  typedef boost::shared_ptr<PointCloudXYZI> PointCloudXYZIPtr;
#endif


} // namespace quanergy


#endif
