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

// parsers for the data packets we want to support
#include <quanergy/parsers/data_packet_parser_00.h>
#include <quanergy/parsers/data_packet_parser_01.h>
#include <quanergy/parsers/data_packet_parser_failover.h>

// module to apply horizontal angle correction
#include <quanergy/modules/horizontal_angle_correction.h>

// conversion module from polar to Cartesian
#include <quanergy/modules/polar_to_cart_converter.h>

void usage(char** argv)
{
  std::cout << "usage: " << argv[0]
      << " --host <host> [-h | --help]" << std::endl << std::endl
      << "    --host               hostname or IP address of the sensor" << std::endl
      << "    --h-angle-correct    apply correction to horizontal angle" << std::endl
      << "-h, --help               show this help and exit" << std::endl;
  return;
}

/// FailoverClient allows packets to pass through that don't have a header (for old M8 data)
typedef quanergy::client::FailoverClient ClientType;
typedef quanergy::client::VariadicPacketParser<quanergy::PointCloudHVDIRPtr,   // return type
                                               quanergy::client::DataPacketParserFailover, // following are data packet types
                                               quanergy::client::DataPacketParser00,
                                               quanergy::client::DataPacketParser01> ParserType;
typedef quanergy::client::PacketParserModule<ParserType> ParserModuleType;
typedef quanergy::client::HorizontalAngleCorrection HCorrectionType;
typedef quanergy::client::PolarToCartConverter ConverterType;

int main(int argc, char** argv)
{
  // get host
  if (argc < 2 || argc > 6 || pcl::console::find_switch(argc, argv, "-h") ||
      pcl::console::find_switch(argc, argv, "--help") || !pcl::console::find_switch(argc, argv, "--host"))
  {
    usage (argv);
    return (0);
  }

  std::string host;
  std::string port = "4141";
  double amplitude = 0.;
  double phase_offset = 0.;

  pcl::console::parse_argument(argc, argv, "--host", host);

  // check for horizontal angle correction arguments
  bool correct_horizontal_angle = false;
  int h_correct_position = pcl::console::find_argument(argc, argv, "--h-angle-correct");
  if (-1 != h_correct_position)
  {
    // 6 arguments are required if we're correcting the horizontal angle
    if (argc < 6)
    {
      usage(argv);
      return (0);
    }

    std::cout << "Correction for horizontal angle offset" << std::endl;
    amplitude = std::stod(argv[h_correct_position+1]);
    phase_offset = std::stod(argv[h_correct_position+2]);

    correct_horizontal_angle = true;
  }

  // create modules
  ClientType client(host, port, 100);
  ParserModuleType parser;
  ConverterType converter;
  VisualizerModule visualizer;

  // setup modules
  parser.get<0>().setFrameId("quanergy");
  parser.get<1>().setFrameId("quanergy");
  parser.get<1>().setReturnSelection(quanergy::client::ReturnSelection::MAX);
  parser.get<2>().setFrameId("quanergy");

  // connect modules
  std::vector<boost::signals2::connection> connections;
  connections.push_back(client.connect([&parser](const ClientType::ResultType& pc){ parser.slot(pc); }));

  // if an amplitude and phase offset are specified, we want an HCorrectionType
  // between the parser and the converter. Otherwise, we want want to connect
  // the parser directly to the converter
  if (correct_horizontal_angle)
  {
    HCorrectionType hcorrector(amplitude, phase_offset);
    connections.push_back(parser.connect([&hcorrector](const ParserModuleType::ResultType& pc){ hcorrector.slot(pc); }));
    connections.push_back(hcorrector.connect([&converter](const HCorrectionType::ResultType& pc){ converter.slot(pc); }));
  }
  else
  {
    connections.push_back(parser.connect([&converter](const ParserModuleType::ResultType& pc){ converter.slot(pc); }));
  }

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
