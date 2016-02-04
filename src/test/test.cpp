/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <mutex>

/*!  The PCL_NO_PRECOMPILE is needed to pull in the PCL cloud geometry
 *   handlers template code for some reason.
 */
#define PCL_NO_PRECOMPILE

// pcl visualizer and parser
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/point_cloud_geometry_handlers.h>
#include <pcl/console/parse.h>

// client; failover adds support for old M8 data
#include <quanergy/parsers/failover_client.h>

// point cloud generation for the two types we support
#include <quanergy/parsers/deserialize_00.h>
#include <quanergy/parsers/pointcloud_generator_00.h>
#include <quanergy/parsers/deserialize_01.h>
#include <quanergy/parsers/pointcloud_generator_01.h>

// coversion module from polar to Cartesian
#include <quanergy/modules/polar_to_cart_converter.h>


void usage(char** argv)
{
  std::cout << "usage: " << argv[0]
      << " -host <host> [-h | --help]" << std::endl
      << "-host hostname or IP address of the sensor" << std::endl;
  std::cout << "\t-h | --help : shows this help and exit" << std::endl;
  return;
}

/** \brief struct contains the intelligence of the test application */
struct Test {

  /// FailoverClient adds a failover to old M8 data
  typedef quanergy::client::FailoverClient<quanergy::client::DataPacket01, quanergy::client::DataPacket00> ClientType;
  typedef pcl::visualization::PCLVisualizer Visualizer;
  typedef quanergy::client::PolarToCartConverter Converter;

  Test(std::string const & host, std::string const & port)
    : client_(host, port, "test frame", 100)
    , viewer_("Cloud Viewer")
  {
    kill_prog_ = false;
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

    /// connect client Polar output to the PolarToCartConverter
    cloud_connection_ = client_.connect([this](const ClientType::Result& pc) 
                                        { this->converter_.slot(pc); } );

    /// connect PolarToCartConverter Cartesian output to this lambda to update the visualization
    converter_connection_ = converter_.connect([this](const Converter::Result& pc)
                                               {      
                                                 std::unique_lock<std::mutex> lock(this->pc_mutex_);
                                                 new_point_cloud_ = pc;
                                               });
  }

  bool newCloud()
  {
    std::unique_lock<std::mutex> lock(pc_mutex_);
    if (new_point_cloud_ != point_cloud_)
    {
      point_cloud_ = new_point_cloud_;
      return true;
    }
    return false;
  }


  /** \brief run the application */
  void run()
  {
    /// thread for client to run on
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

    

    /// spin loop for updating visualizer
    while (!viewer_.wasStopped() && !kill_prog_)
    {
      if (newCloud())
      {
        pcl::visualization::PointCloudColorHandlerGenericField<quanergy::PointXYZIR> color_handler(point_cloud_, "intensity");
        if (!this->viewer_.updatePointCloud<quanergy::PointXYZIR>(point_cloud_, color_handler, "Quanergy"))
        {
          this->viewer_.addPointCloud<quanergy::PointXYZIR>(point_cloud_, color_handler, "Quanergy");
        }
      }
      viewer_.spinOnce();
    }

    /// shut things down
    converter_connection_.disconnect();
    cloud_connection_.disconnect();
    client_.stop();
    client_thread.join();
  }

private:

  ClientType client_;
  boost::signals2::connection cloud_connection_;
  boost::signals2::connection converter_connection_;

  std::mutex pc_mutex_;

  Visualizer viewer_;
  Converter converter_;

  Converter::Result new_point_cloud_;
  Converter::Result point_cloud_;

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

  Test test(host, port);

  test.run();

  return (0);
}
