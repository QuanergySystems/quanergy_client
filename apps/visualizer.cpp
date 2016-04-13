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

// module to apply encoder correction
#include <quanergy/modules/encoder_correction.h>
#include <quanergy/modules/encoder_angle_calibration.h>

// conversion module from polar to Cartesian
#include <quanergy/modules/polar_to_cart_converter.h>

namespace
{
  static const std::string AMPLITUDE_STR{"--encoder-amplitude-correct"};
  static const std::string PHASE_STR{"--encoder-phase-correct"};
  static const std::string CALIBRATE_STR{"--calibrate"};
}

void usage(char** argv)
{
  std::cout << "usage: " << argv[0]
      << " --host <host> [-h | --help] [" << AMPLITUDE_STR << " <value> " << PHASE_STR << " <value>]" << std::endl << std::endl

      << "    --host                          hostname or IP address of the sensor" << std::endl
      << "    " << AMPLITUDE_STR << "  apply encoder correction with user-defined amplitude" << std::endl
      << "    " << PHASE_STR << "      apply phase correction with user-defined amplitude" << std::endl
      << "    " << CALIBRATE_STR << "   calibrate the host sensor and apply calibration to outgoing points" << std::endl
      << "-h, --help                          show this help and exit" << std::endl;
  return;
}

/// FailoverClient allows packets to pass through that don't have a header (for old M8 data)
typedef quanergy::client::FailoverClient ClientType;
typedef quanergy::client::VariadicPacketParser<quanergy::PointCloudHVDIRPtr,   // return type
                                               quanergy::client::DataPacketParserFailover, // following are data packet types
                                               quanergy::client::DataPacketParser00,
                                               quanergy::client::DataPacketParser01> ParserType;
typedef quanergy::client::PacketParserModule<ParserType> ParserModuleType;
typedef quanergy::client::EncoderCorrection EncoderCorrectionType;
typedef quanergy::calibration::EncoderAngleCalibration CalibrationType;
typedef quanergy::client::PolarToCartConverter ConverterType;

int main(int argc, char** argv)
{
  int max_num_args = 8;
  // get host
  if (argc < 2 || argc > max_num_args || pcl::console::find_switch(argc, argv, "-h") ||
      pcl::console::find_switch(argc, argv, "--help") || !pcl::console::find_switch(argc, argv, "--host"))
  {
    usage (argv);
    return (0);
  }

  // if only one of the encoder correction parameters is specified, return
  // usage statement
  if ((pcl::console::find_switch(argc, argv, AMPLITUDE_STR.c_str()) &&
       !pcl::console::find_switch(argc, argv, PHASE_STR.c_str())) ||
      (!pcl::console::find_switch(argc, argv, AMPLITUDE_STR.c_str()) &&
       pcl::console::find_switch(argc, argv, PHASE_STR.c_str())))
  {
    usage(argv);
    return (1);
  }

  std::string host;
  std::string port = "4141";
  double amplitude = 0.;
  double phase_offset = 0.;

  pcl::console::parse_argument(argc, argv, "--host", host);

  // check for encoder correction arguments
  bool correct_encoder_angle = false;
  if (pcl::console::find_switch(argc, argv, AMPLITUDE_STR.c_str()) &&
      pcl::console::find_switch(argc, argv, PHASE_STR.c_str()))
  {
    pcl::console::parse_argument(argc, argv, AMPLITUDE_STR.c_str(), amplitude);
    pcl::console::parse_argument(argc, argv, PHASE_STR.c_str(), phase_offset);
    correct_encoder_angle = true;
  }


  // create modules
  ClientType client(host, port, 100);
  ParserModuleType parser;
  ConverterType converter;
  VisualizerModule visualizer;

  EncoderCorrectionType encoder_corrector(amplitude, phase_offset);
  CalibrationType calibrator;

  // setup modules
  parser.get<0>().setFrameId("quanergy");
  parser.get<1>().setFrameId("quanergy");
  parser.get<1>().setReturnSelection(quanergy::client::ReturnSelection::MAX);
  parser.get<2>().setFrameId("quanergy");

  // connect modules
  std::vector<boost::signals2::connection> connections;

  auto run_client = [&client, &visualizer]
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
  };

  std::thread client_thread;

  if (pcl::console::find_switch(argc, argv, CALIBRATE_STR.c_str()))
  {
    connections.push_back(client.connect([&parser](const ClientType::ResultType& pc){ parser.slot(pc); }));
    connections.push_back(parser.connect([&calibrator](const ParserModuleType::ResultType& pc){ calibrator.slot(pc); }));
    connections.push_back(calibrator.connect([&converter](const CalibrationType::ResultType& pc){ converter.slot(pc); }));
    connections.push_back(converter.connect([&visualizer](const ConverterType::ResultType& pc){ visualizer.slot(pc); }));

    // run the client with the calibrator and wait for a signal from the
    // calibrator that a successful calibration has been performed
    client_thread = std::thread(run_client);
    
  }
  else
  {

    if (correct_encoder_angle)
    {
      connections.push_back(client.connect([&parser](const ClientType::ResultType& pc){ parser.slot(pc); }));
      connections.push_back(parser.connect([&encoder_corrector](const ParserModuleType::ResultType& pc){ encoder_corrector.slot(pc); }));
      connections.push_back(encoder_corrector.connect([&converter](const EncoderCorrectionType::ResultType& pc){ converter.slot(pc); }));
      connections.push_back(converter.connect([&visualizer](const ConverterType::ResultType& pc){ visualizer.slot(pc); }));
    }
    else
    {
      connections.push_back(client.connect([&parser](const ClientType::ResultType& pc){ parser.slot(pc); }));
      connections.push_back(parser.connect([&converter](const ParserModuleType::ResultType& pc){ converter.slot(pc); }));
      connections.push_back(converter.connect([&visualizer](const ConverterType::ResultType& pc){ visualizer.slot(pc); }));
    }

    // start client on a separate thread
    client_thread = std::thread(run_client);

  } // end of not calibrating

  // start visualizer (blocks until stopped)
  visualizer.run();

  // clean up
  client.stop();
  connections.clear();
  client_thread.join();

  return (0);
}
