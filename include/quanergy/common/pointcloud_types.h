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

#include <memory>
#include <boost/shared_ptr.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <quanergy/common/point_xyzir.h>
#include <quanergy/common/point_hvdir.h>

namespace quanergy 
{
  /*!
   *  Note the PCL uses boost shared pointers internally.
   *  @TODO: Switch to std::shared_ptr?
   */

  typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;
  typedef pcl::PointCloud<quanergy::PointXYZIR> PointCloudXYZIR;
  typedef pcl::PointCloud<quanergy::PointHVDIR> PointCloudHVDIR;

#if 0
  typedef std::shared_ptr<PointCloudXYZI> PointCloudXYZIPtr;
  typedef std::shared_ptr<PointCloudXYZIR> PointCloudXYZIRPtr;
  typedef std::shared_ptr<PointCloudHVDIR> PointCloudHDVIRPtr;

  typedef std::shared_ptr<PointCloudXYZI const> PointCloudXYZIConstPtr;
  typedef std::shared_ptr<PointCloudXYZIR const> PointCloudXYZIRConstPtr;
  typedef std::shared_ptr<PointCloudHVDIR const> PointCloudHDVIRConstPtr;
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
