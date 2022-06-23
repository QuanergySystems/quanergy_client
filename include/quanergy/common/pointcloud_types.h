/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/**  \file pointcloud_types.h
  *
  *  \brief Provide typedefs for standard point clouds.
  */

#ifndef QUANERGY_CLIENT_POINTCLOUD_TYPES_H
#define QUANERGY_CLIENT_POINTCLOUD_TYPES_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#if PCL_VERSION_COMPARE(<, 1, 10, 0)
#include <boost/shared_ptr.hpp>
#endif

#include <quanergy/common/point_xyzir.h>
#include <quanergy/common/point_hvdir.h>

namespace quanergy 
{
  typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;
  typedef pcl::PointCloud<quanergy::PointXYZIR> PointCloudXYZIR;
  typedef pcl::PointCloud<quanergy::PointHVDIR> PointCloudHVDIR;

#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
  typedef pcl::shared_ptr<PointCloudXYZI> PointCloudXYZIPtr;
  typedef pcl::shared_ptr<PointCloudXYZIR> PointCloudXYZIRPtr;
  typedef pcl::shared_ptr<PointCloudHVDIR> PointCloudHVDIRPtr;

  typedef pcl::shared_ptr<PointCloudXYZI const> PointCloudXYZIConstPtr;
  typedef pcl::shared_ptr<PointCloudXYZIR const> PointCloudXYZIRConstPtr;
  typedef pcl::shared_ptr<PointCloudHVDIR const> PointCloudHVDIRConstPtr;
#else
  typedef boost::shared_ptr<PointCloudXYZI> PointCloudXYZIPtr;
  typedef boost::shared_ptr<PointCloudXYZIR> PointCloudXYZIRPtr;
  typedef boost::shared_ptr<PointCloudHVDIR> PointCloudHVDIRPtr;

  typedef boost::shared_ptr<PointCloudXYZI const> PointCloudXYZIConstPtr;
  typedef boost::shared_ptr<PointCloudXYZIR const> PointCloudXYZIRConstPtr;
  typedef boost::shared_ptr<PointCloudHVDIR const> PointCloudHVDIRConstPtr;
#endif

} // namespace quanergy


#endif
