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
#include <quanergy/parsers/pointcloud_generator_failover.h>

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

/// FailoverClient allows packets to pass through that don't have a header (for old M8 data)
typedef quanergy::client::FailoverClient ClientType;
/// templates are return type and then packets to support
typedef quanergy::client::VariadicPacketParser<quanergy::PointCloudHVDIRPtr, quanergy::client::M8DataPacket, quanergy::client::DataPacket00, quanergy::client::DataPacket01> ParserType;
typedef quanergy::client::PacketParserModule<ParserType> ParserModuleType;
typedef quanergy::client::PolarToCartConverter ConverterType;

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
  ClientType client(host, port, 100);
  ParserModuleType parser;
  ConverterType converter;
  VisualizerModule visualizer;

  // connect modules
  std::vector<boost::signals2::connection> connections;
  connections.push_back(client.connect([&parser](const ClientType::ResultType& pc){ parser.slot(pc); }));
  connections.push_back(parser.connect([&converter](const ParserModuleType::ResultType& pc){ converter.slot(pc); }));
  connections.push_back(converter.connect([&visualizer](const ConverterType::ResultType& pc){ visualizer.slot(pc); }));

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
