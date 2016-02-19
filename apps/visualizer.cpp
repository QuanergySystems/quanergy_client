/****************************************************************
 **                                                            **
 **  Copyright(C) 2016 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

// visualization module
#include "visualizer_module.h"

// console parser
#include <pcl/console/parse.h>

// client; failover adds support for old M8 data
#include <quanergy/client/failover_client.h>

// point cloud generation for the two types we support
#include <quanergy/parsers/data_packet_00.h>
#include <quanergy/parsers/pointcloud_generator_00.h>
#include <quanergy/parsers/data_packet_01.h>
#include <quanergy/parsers/pointcloud_generator_01.h>

// conversion module from polar to Cartesian
#include <quanergy/modules/polar_to_cart_converter.h>

void usage(char** argv)
{
  std::cout << "usage: " << argv[0]
      << " --host <host> [-h | --help]" << std::endl
      << "--host hostname or IP address of the sensor" << std::endl;
  std::cout << "\t-h | --help : shows this help and exit" << std::endl;
  return;
}

/// FailoverClient adds a failover to old M8 data
typedef quanergy::client::FailoverClient<quanergy::client::DataPacket01, quanergy::client::DataPacket00> ClientType;
typedef quanergy::client::PolarToCartConverter Converter;

int main(int argc, char** argv)
{
  // get host
  if (argc < 2 || argc > 3 || pcl::console::find_switch(argc, argv, "-h") ||
      pcl::console::find_switch(argc, argv, "--help") || !pcl::console::find_switch(argc, argv, "--host"))
  {
    usage (argv);
    return (0);
  }

  std::string host;
  std::string port = "4141";

  pcl::console::parse_argument(argc, argv, "--host", host);

  // create modules
  ClientType client(host, port, "test frame", 100);
  Converter converter;
  VisualizerModule visualizer;

  // connect modules
  std::vector<boost::signals2::connection> connections;
  connections.push_back(client.connect([&converter](const ClientType::Result& pc){ converter.slot(pc); }));
  connections.push_back(converter.connect([&visualizer](const Converter::Result& pc){ visualizer.slot(pc); }));

  // start client on a separate thread
  std::thread client_thread([&client, &visualizer]
                            {
                              try
                              {
                                client.run();
                              }
                              catch (std::exception& e)
                              {
                                std::cerr << "Terminating after catching exception: " << e.what() << std::endl;
                                visualizer.stop();
                              }
                            });


  // start visualizer (blocks until stopped)
  visualizer.run();

  // clean up
  client.stop();
  connections.clear();
  client_thread.join();

  return (0);
}
