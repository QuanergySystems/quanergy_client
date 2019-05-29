/****************************************************************
 **                                                            **
 **  Copyright(C) 2018 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

// control flow management
#include <mutex>
#include <condition_variable>

// console parser
#include <pcl/console/parse.h>

#include <quanergy/client/sensor_client.h>

// parsers for the data packets we want to support
#include <quanergy/parsers/data_packet_parser_00.h>
#include <quanergy/parsers/data_packet_parser_01.h>
#include <quanergy/parsers/data_packet_parser_04.h>
#include <quanergy/parsers/variadic_packet_parser.h>

// module to apply encoder correction
#include <quanergy/modules/encoder_angle_calibration.h>

// conversion module from polar to Cartesian
#include <quanergy/modules/polar_to_cart_converter.h>

// define some strings that will be used on command line
namespace
{
  static const std::string MANUAL_CORRECT{"--manual-correct"};
  static const std::string CALIBRATE_STR{"--calibrate"};
  static const std::string AMPLITUDE_STR{"--amplitude"};
  static const std::string PHASE_STR{"--phase"};
}

// output usage message
void usage(char** argv)
{
  std::cout << "usage: " << argv[0]
            << " --host <host> [-h | --help] [" << CALIBRATE_STR << "][" << MANUAL_CORRECT << " " << AMPLITUDE_STR << " <value> " << PHASE_STR << " <value>]" << std::endl << std::endl

            << "    --host        hostname or IP address of the sensor" << std::endl
            << "    " << CALIBRATE_STR << "   calibrate the host sensor and apply calibration to outgoing points" << std::endl
            << "    " << MANUAL_CORRECT << " --amplitude <amplitude> --phase <phase>    Manually correct encoder error specifying amplitude and phase correction, in radians" << std::endl
            << "-h, --help        show this help and exit" << std::endl;
  return;
}

// convenient typedefs
// SensorClient connects to any quanergy sensor; it pulls TCP packets off the stream, queues them, and then signals them on a separate thread
typedef quanergy::client::SensorClient ClientType;
// VariadicPacketParser produces a polar point cloud; it uses information in the packet to find the correct parser from the ones supplied
typedef quanergy::client::VariadicPacketParser<quanergy::PointCloudHVDIRPtr,                      // return type
                                               quanergy::client::DataPacketParser00,              // PARSER_00_INDEX
                                               quanergy::client::DataPacketParser01,              // PARSER_01_INDEX
                                               quanergy::client::DataPacketParser04> ParserType;  // PARSER_04_INDEX

// enum to make indexing into the VariadicPacketParser easier
enum
{
  PARSER_00_INDEX = 0,
  PARSER_01_INDEX = 1,
  PARSER_04_INDEX = 2
};

// more typedefs
// PacketParserModule wraps the parser with signal/slot functionality
typedef quanergy::client::PacketParserModule<ParserType> ParserModuleType;
// EncoderAngleCalibration provides some calibration functionality for M8
typedef quanergy::calibration::EncoderAngleCalibration CalibrationType;
// PolarToCartConverter converts the polar point cloud to Cartesian
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

  std::string host;
  std::string port = "4141";

  pcl::console::parse_argument(argc, argv, "--host", host);

  // create modules
  ClientType client(host, port, 100);
  ParserModuleType parser;
  ConverterType converter;

  CalibrationType calibrator;

  // setup modules
  // setFrameId sets the frame id in the PCL point clouds
  parser.get<PARSER_00_INDEX>().setFrameId("quanergy");
  // setReturnSelection allows choosing between the 3 returns (Packet00, only)
  parser.get<PARSER_00_INDEX>().setReturnSelection(0);
  // setDegreesOfSweepPerCloud allows breaking the point clouds into smaller pieces (M8 only)
  parser.get<PARSER_00_INDEX>().setDegreesOfSweepPerCloud(360.0);
  parser.get<PARSER_01_INDEX>().setFrameId("quanergy");
  parser.get<PARSER_04_INDEX>().setFrameId("quanergy");

  // store connections for cleaner shutdown
  std::vector<boost::signals2::connection> connections;

  // connect the packets from the client to the parser
  connections.push_back(client.connect([&parser](const ClientType::ResultType& pc){ parser.slot(pc); }));
  
  if (pcl::console::find_switch(argc, argv, CALIBRATE_STR.c_str()) &&
      pcl::console::find_switch(argc, argv, MANUAL_CORRECT.c_str()))
  {
    std::cout << "Cannot perform calibration and apply manual correction simultaneously." << std::endl;
    usage(argv);
    return(0);
  }

  // if we're doing automatic calibration or if we're setting the calibration
  // manually, include the calibrator in the chain
  if (pcl::console::find_switch(argc, argv, CALIBRATE_STR.c_str()) ||
      pcl::console::find_switch(argc, argv, MANUAL_CORRECT.c_str()))
  {
    // connect the parser to the calibrator
    connections.push_back(parser.connect([&calibrator](const ParserModuleType::ResultType& pc){ calibrator.slot(pc); }));
    // connect the calibrator to the converter
    connections.push_back(calibrator.connect([&converter](const CalibrationType::ResultType& pc){ converter.slot(pc); }));

    // set calibrator parameters
    if (pcl::console::find_switch(argc, argv, MANUAL_CORRECT.c_str()))
    {
      if (!pcl::console::find_switch(argc, argv, AMPLITUDE_STR.c_str()) ||
          !pcl::console::find_switch(argc, argv, PHASE_STR.c_str()))
      {
        usage(argv);
        return(0);
      }
      double amplitude = 0.;
      double phase = 0.;
      pcl::console::parse_argument(argc, argv, AMPLITUDE_STR.c_str(), amplitude);
      pcl::console::parse_argument(argc, argv, PHASE_STR.c_str(), phase);

      calibrator.setParams(amplitude, phase);
    }
  }
  else
  {
    // connect the parser to the converter
    connections.push_back(parser.connect([&converter](const ParserModuleType::ResultType& pc){ converter.slot(pc); }));
  }

  ////////////////////////////////////////////
  /// connect application specific logic here to consume the point cloud
  ////////////////////////////////////////////
  unsigned int cloud_count = 0;
  connections.push_back(converter.connect([&cloud_count](const ConverterType::ResultType& /*pc*/){ ++cloud_count; if(cloud_count % 100 == 0) std::cout << "clouds received: " << cloud_count << std::endl; }));

  // variables to help with control flow
  std::mutex state_mutex;
  std::condition_variable state_condition;
  enum struct State {RUN, STOP, EXIT};
  State state(State::STOP);
  
  // run the client in a separate thread
  std::thread client_thread([&client, &state, &state_condition, &state_mutex]
  {
    while (true)
    {
      try
      {
        {
          // wait for state to be change from STOP; this is scoped for the lock
          std::unique_lock<std::mutex> lk(state_mutex);
          state_condition.wait(lk, [&state]{return state != State::STOP;});

          if (state == State::EXIT)
          {
            break; // exit while loop
          }
        }

        client.run();
      }      
      catch (std::exception& e)
      {
        std::cerr << "Caught exception (" << e.what() << "); stopping" << std::endl;
        std::lock_guard<std::mutex> lk(state_mutex);
        state = State::STOP;
      }
    }
  });

  while (state != State::EXIT)
  {
    // get input
    ////////////////////////////////////////
    /// put application specific logic here
    ////////////////////////////////////////
    std::string input;
    std::cout << "Enter 'run' to connect, 'stop' to disconnect, or 'exit' to exit the program" << std::endl;
    std::getline(std::cin, input);
    
    {
      // set the state and take appropriate action; this is scoped for the lock
      std::lock_guard<std::mutex> lk(state_mutex);
      if (input == "run")
      {
        state = State::RUN;
      }
      else if (input == "stop")
      {
        state = State::STOP;
        client.stop();

        if (pcl::console::find_switch(argc, argv, CALIBRATE_STR.c_str()))
        {
          calibrator.reset();
        }
      }
      else if (input == "exit")
      {
        state = State::EXIT;
        client.stop();
      }
      else
      {
        std::cout << "Input (" << input << ") doesn't match any of the accepted options" << std::endl;
      }
    }

    // notify the condition variable to stop waiting
    state_condition.notify_one();
  }

  // clean up
  client.stop();
  connections.clear();
  client_thread.join();

  return (0);
}
