/****************************************************************
 **                                                            **
 **  Copyright(C) 2016 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include "visualizer_module.h"

#include <chrono>
#include <thread>

VisualizerModule::VisualizerModule()
  : viewer_("Cloud Viewer")
{
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
  if (pc_mutex_.try_lock_for(std::chrono::milliseconds(10)))
  {
    if (!viewer_.updatePointCloud<quanergy::PointXYZIR>(new_cloud, color_handler, new_cloud->header.frame_id))
    {
      viewer_.addPointCloud<quanergy::PointXYZIR>(new_cloud, color_handler, new_cloud->header.frame_id);
    }

    pc_mutex_.unlock();
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
