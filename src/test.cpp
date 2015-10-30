/****************************************************************************
 **
 ** Copyright (C) 2014-- Quanergy Systems. All Rights Reserved.
 ** Contact: http://www.quanergy.com
 **
 ****************************************************************************/

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/console/parse.h>

#include <quanergy/client/deserialize_00.h>
#include <quanergy/client/pointcloud_generator_00.h>
#include <quanergy/client/deserialize_01.h>
#include <quanergy/client/pointcloud_generator_01.h>
#include <quanergy/client/failover_client.h>


void usage(char** argv)
{
  std::cout << "usage: " << argv[0]
      << " -host <host> [-port <port>] [-h | --help]" << std::endl
      << "-host hostname or IP address of the sensor" << std::endl
      << "-port TCP port used by the sensor 4141[default]" << std::endl;
  std::cout << "\t-h | --help : shows this help and exit" << std::endl;
  return;
}

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

  /// FailoverClient adds a failover to old M8 data
  typedef quanergy::FailoverClient<quanergy::DataPacket01, quanergy::DataPacket00> ClientType;
  ClientType client(host, port);
  boost::signals2::connection cloud_connection;

  pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

#if (PCL_MAJOR_VERSION == 1 && PCL_MINOR_VERSION == 7 && PCL_REVISION_VERSION <= 2)
  viewer.addCoordinateSystem(1.0);
#else
  viewer.addCoordinateSystem (1.0, "global");
#endif
  viewer.setBackgroundColor (0, 0, 0);
  viewer.initCameraParameters ();
  viewer.setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
  viewer.setCameraClipDistances (0.0, 50.0);

  cloud_connection = client.connect([&viewer](const ClientType::Result& pc)
                                {
                                  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler(pc,"intensity");
                                  if (!viewer.updatePointCloud<pcl::PointXYZI>(pc, color_handler, "Quanergy"))
                                    viewer.addPointCloud<pcl::PointXYZI>(pc, color_handler, "Quanergy");
                                  viewer.spinOnce();
                                });

  // run client on separate thread
  std::atomic_bool kill_prog(false);
  std::thread client_thread([&client, &kill_prog]
        {
          try
          {
            client.run();
          }
          catch (std::exception& e)
          {
            std::cerr << "Terminating after catching exception: " << e.what() << std::endl;
            kill_prog = true;
          }
        });

  while (!viewer.wasStopped() && !kill_prog);

  cloud_connection.disconnect();
  client.stop();
  client_thread.join();

  return (0);
}
