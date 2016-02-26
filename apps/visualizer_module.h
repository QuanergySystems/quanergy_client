/****************************************************************
 **                                                            **
 **  Copyright(C) 2016 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#ifndef QUANERGY_CLIENT_VISUALIZER_MODULE_H
#define QUANERGY_CLIENT_VISUALIZER_MODULE_H

#include <mutex>

/*!  The PCL_NO_PRECOMPILE is needed to pull in the PCL cloud geometry
 *   handlers template code for some reason.
 */
#define PCL_NO_PRECOMPILE

// pcl visualizer
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/point_cloud_geometry_handlers.h>

#include <quanergy/common/pointcloud_types.h>

/** \brief struct contains the intelligence of the test application */
class VisualizerModule
{
public:
  VisualizerModule();

  void slot(const quanergy::PointCloudXYZIRConstPtr& new_cloud);

  /** \brief run the application */
  void run();

  /** \brief stop the application */
  void stop();

private:
  std::timed_mutex pc_mutex_;

  pcl::visualization::PCLVisualizer viewer_;
};

#endif
