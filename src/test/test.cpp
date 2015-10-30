/****************************************************************************
 **
 ** Copyright (C) 2014-- Quanergy Systems. All Rights Reserved.
 ** Contact: http://www.quanergy.com
 **
 ****************************************************************************/

#include <mutex>

/*!  The PCL_NO_PRECOMPILE is needed to pull in the PCL cloud geometry
 *   handlers template code for some reason.
 */
#define PCL_NO_PRECOMPILE

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/point_cloud_geometry_handlers.h>
#include <pcl/console/parse.h>

#include <quanergy/client/deserialize_00.h>
#include <quanergy/client/pointcloud_generator_00.h>
#include <quanergy/client/deserialize_01.h>
#include <quanergy/client/pointcloud_generator_01.h>
#include <quanergy/client/failover_client.h>
#include <quanergy/client/polar_to_cart_filter.h>


void usage(char** argv)
{
  std::cout << "usage: " << argv[0]
      << " -host <host> [-port <port>] [-h | --help]" << std::endl
      << "-host hostname or IP address of the sensor" << std::endl
      << "-port TCP port used by the sensor 4141[default]" << std::endl;
  std::cout << "\t-h | --help : shows this help and exit" << std::endl;
  return;
}



struct Test {

  /// FailoverClient adds a failover to old M8 data
  typedef quanergy::client::FailoverClient<quanergy::client::DataPacket01, quanergy::client::DataPacket00> ClientType;
  typedef pcl::visualization::PCLVisualizer Visualizer;
  typedef quanergy::client::PolarToCartFilter Filter;

  Test(std::string const & host, std::string const & port)
    : client_(host, port, "test frame") 
    , viewer_("Cloud Viewer")
    , kill_prog_(false)
  {

#if (PCL_MAJOR_VERSION == 1 && PCL_MINOR_VERSION == 7 && PCL_REVISION_VERSION <= 2)
    viewer_.addCoordinateSystem(1.0);
#else
    viewer_.addCoordinateSystem (1.0, "global");
#endif
    viewer_.setBackgroundColor (0, 0, 0);
    viewer_.initCameraParameters ();
    viewer_.setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
    viewer_.setCameraClipDistances (0.0, 50.0);

    cloud_connection_ = client_.connect([this](const ClientType::Result& pc) 
                                        { this->filter_.slot(pc); } );

    filter_connection_ = filter_.connect([this](const Filter::Result& pc)
                                         {
                                           pcl::visualization::PointCloudColorHandlerGenericField<quanergy::PointXYZIR> color_handler(pc,"intensity");
                                           if (!this->viewer_.updatePointCloud<quanergy::PointXYZIR>(pc, color_handler, "Quanergy"))
                                           {
                                             std::unique_lock<std::mutex> lock(this->viewer_spin_mutex_);
                                             this->viewer_.addPointCloud<quanergy::PointXYZIR>(pc, color_handler, "Quanergy");
                                           }
                                         });
  }

  void run()
  {
    std::thread client_thread([this]
                              {
                                try
                                {
                                  this->client_.run();
                                }
                                catch (std::exception& e)
                                {
                                  std::cerr << "Terminating after catching exception: " << e.what() << std::endl;
                                  this->kill_prog_ = true;
                                }
                              });

    while (!viewer_.wasStopped() && !kill_prog_)
    {
      std::unique_lock<std::mutex> lock(viewer_spin_mutex_);
      viewer_.spinOnce();
    }

    filter_connection_.disconnect();
    cloud_connection_.disconnect();
    client_.stop();
    client_thread.join();
  }

private:

  ClientType client_;
  boost::signals2::connection cloud_connection_;
  boost::signals2::connection filter_connection_;

  std::mutex viewer_spin_mutex_;

  Visualizer viewer_;
  Filter filter_;

  // run client on separate thread
  std::atomic_bool kill_prog_;
};


int main(int argc, char** argv)
{
  if (argc < 2 || argc > 3 || pcl::console::find_switch(argc, argv, "-h") ||
      pcl::console::find_switch(argc, argv, "--help"))
  {
    usage (argv);
    return (0);
  }

  std::string host;
  std::string port = "4141";

  pcl::console::parse_argument (argc, argv, "-host", host);
  pcl::console::parse_argument (argc, argv, "-port", port);

  Test test(host, port);

  test.run();

  return (0);
}
