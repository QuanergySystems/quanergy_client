/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include "visualizer_module.h"

#include <chrono>
#include <thread>

VisualizerModule::VisualizerModule()
  : viewer_("Cloud Viewer")
{
#if (VTK_MAJOR_VERSION == 9 && ((VTK_MINOR_VERSION == 0 && VTK_BUILD_VERSION != 0) || (VTK_MINOR_VERSION == 1 && VTK_BUILD_VERSION == 0)) \
    && (PCL_MAJOR_VERSION < 1 || (PCL_MAJOR_VERSION == 1 && (PCL_MINOR_VERSION < 12 || (PCL_MINOR_VERSION == 12 && PCL_REVISION_VERSION <= 1)))))
  // VTK versions 9.0.1 to 9.1.0 have a bug making spin_once not work with PCL
  // newer versions of VTK and PCL should fix the issue (PCL implemented a workaround)
  // for these versions, we'll throw a meaningful error instead of getting Segfault
  // https://github.com/PointCloudLibrary/pcl/issues/5237
  throw std::runtime_error("A bug in VTK, makes this application unable to run; see apps/visualizer_module.cpp for more information");
#endif

  /// basic visualization setup
#if (PCL_MAJOR_VERSION == 1 && PCL_MINOR_VERSION == 7 && PCL_REVISION_VERSION <= 2)
  viewer_.addCoordinateSystem(1.0);
#else
  viewer_.addCoordinateSystem (1.0, "global");
#endif
  viewer_.setBackgroundColor (0, 0, 0);
  viewer_.initCameraParameters ();
  viewer_.setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
  viewer_.setCameraClipDistances (0.0, 50.0);
}

void VisualizerModule::slot(const quanergy::PointCloudXYZIRConstPtr& new_cloud)
{
  pcl::visualization::PointCloudColorHandlerGenericField<quanergy::PointXYZIR> color_handler(new_cloud, "intensity");

  // don't block if visualizer isn't available
  std::unique_lock<std::timed_mutex> lock(pc_mutex_, std::chrono::milliseconds(10));
  if (lock)
  {
    if (!viewer_.updatePointCloud<quanergy::PointXYZIR>(new_cloud, color_handler, new_cloud->header.frame_id))
    {
      viewer_.addPointCloud<quanergy::PointXYZIR>(new_cloud, color_handler, new_cloud->header.frame_id);
    }
  }
}

/** \brief run the application */
void VisualizerModule::run()
{
  /// spin loop for updating visualizer
  while (!viewer_.wasStopped())
  {
    std::unique_lock<std::timed_mutex> lock(pc_mutex_);
    viewer_.spinOnce();
    lock.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

/** \brief stop the application */
void VisualizerModule::stop()
{
  viewer_.close();
}
