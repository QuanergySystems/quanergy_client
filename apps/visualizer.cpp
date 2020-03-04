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

// TCP client for sensor
#include <quanergy/client/sensor_client.h>

// HTTP client to get calibration info from sensors
#include <quanergy/client/http_client.h>

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

  // validate arguments
  if (argc < 2 || argc > max_num_args || pcl::console::find_switch(argc, argv, "-h") ||
      pcl::console::find_switch(argc, argv, "--help") || !pcl::console::find_switch(argc, argv, "--host"))
  {
    usage(argv);
    return(0);
  }

  if (pcl::console::find_switch(argc, argv, CALIBRATE_STR.c_str()) &&
      pcl::console::find_switch(argc, argv, MANUAL_CORRECT.c_str()))
  {
    std::cout << "Cannot perform calibration and apply manual correction simultaneously." << std::endl;
    usage(argv);
    return(0);
  }

  if (pcl::console::find_switch(argc, argv, MANUAL_CORRECT.c_str()) &&
      (!pcl::console::find_switch(argc, argv, AMPLITUDE_STR.c_str()) ||
        !pcl::console::find_switch(argc, argv, PHASE_STR.c_str())))
  {
    std::cout << "For manual encoder correction, amplitude and phase must be provided." << std::endl;
    usage(argv);
    return(0);
  }

  std::string host;
  std::string port = "4141";

  // get host
  pcl::console::parse_argument(argc, argv, "--host", host);

  // create modules
  ClientType client(host, port, 100);
  quanergy::client::HTTPClient http_client(host);
  ParserModuleType parser;
  ConverterType converter;
  VisualizerModule visualizer;
  CalibrationType calibrator;


  // are we doing encoder calibration?
  bool perform_encoder_calibration = pcl::console::find_switch(argc, argv, CALIBRATE_STR.c_str())
                                     || pcl::console::find_switch(argc, argv, MANUAL_CORRECT.c_str());
  double amplitude = 0.;
  double phase = 0.;

  if (pcl::console::find_switch(argc, argv, MANUAL_CORRECT.c_str()))
  {
    // get calibrator parameters
    pcl::console::parse_argument(argc, argv, AMPLITUDE_STR.c_str(), amplitude);
    pcl::console::parse_argument(argc, argv, PHASE_STR.c_str(), phase);

    // this also turns off the recalculation of calibration params
    calibrator.setParams(amplitude, phase);
  }


  // setup parsers
  // setFrameId sets the frame id in the PCL point clouds
  parser.get<PARSER_00_INDEX>().setFrameId("quanergy");
  // setReturnSelection allows choosing between the 3 returns (Packet00, only)
  parser.get<PARSER_00_INDEX>().setReturnSelection(0);
  // setDegreesOfSweepPerCloud allows breaking the point clouds into smaller pieces (M8 only)
  parser.get<PARSER_00_INDEX>().setDegreesOfSweepPerCloud(360.0);
  parser.get<PARSER_01_INDEX>().setFrameId("quanergy");
  parser.get<PARSER_04_INDEX>().setFrameId("quanergy");


  // get deviceInfo from sensor for calibration
  std::stringstream device_info_stream;
  try
  {
    http_client.read("/PSIA/System/deviceInfo", device_info_stream);
    boost::property_tree::ptree device_info_tree;
    boost::property_tree::read_xml(device_info_stream, device_info_tree);

    // check sensor type
    auto model = device_info_tree.get<std::string>("DeviceInfo.model");
    std::cout << "got model: " << model << std::endl;

    auto calibration_data = device_info_tree.get_child_optional("DeviceInfo.calibration");
    if (!calibration_data)
    {
      if (model.find("MQ") != std::string::npos)
      {
        std::cerr << "ERROR: MQ sensor has no calibration information" << std::endl;
        return -2;
      }
      else if (model.find("M8") != std::string::npos)
      {
        std::cout << "No calibration information available on sensor, proceeding with M8 defaults" << std::endl;

        // tell parsers to use M8 defaults
        parser.get<PARSER_00_INDEX>().setVerticalAngles(quanergy::client::SensorType::M8);
        parser.get<PARSER_04_INDEX>().setVerticalAngles(quanergy::client::SensorType::M8);
      }
      else if (model.find("S3") != std::string::npos)
      {
        // Calibration data is not currently supported on S3, so silently proceed
      }
      else
      {
        std::cerr << "ERROR: unrecognized sensor model" << std::endl;
        return -2;
      }
    }
    else
    {
      std::cout << "got cal" << std::endl;
      if (model.find("MQ") == std::string::npos && model.find("M8") == std::string::npos)
      {
        std::cout << "Found calibration information for an unsupported sensor type; ignoring" << std::endl;
      }
      else
      {
        if (perform_encoder_calibration)
        {
          std::cout << "Command line encoder calibration options override values stored on the sensor." << std::endl;
        }
        else
        {
          // since we have cal values from sensor, we need to include encoder calibration
          perform_encoder_calibration = true;

          // get encoder cal values; they get applied later
          amplitude = calibration_data->get<double>("encoder.amplitude");
          phase = calibration_data->get<double>("encoder.phase");

          // this also turns off the recalculation of calibration params
          calibrator.setParams(amplitude, phase);
        }

        // get vertical angles
        std::vector<double> vertical_angles(
              calibration_data->get<unsigned int>("lasers.<xmlattr>.number"), 0.);

        for (auto& laser : calibration_data->get_child("lasers"))
        {
          if (laser.first == "laser")
          {
            auto id = laser.second.get<unsigned int>("<xmlattr>.id");
            if (id > vertical_angles.size())
            {
              std::cerr << "ERROR: invalid laser id in calibration info" << std::endl;
              return -2;
            }
            else
            {
              vertical_angles[id] = laser.second.get<double>("v");
            }
          }
        }

        // send the vertical angles to the parsers
        parser.get<PARSER_00_INDEX>().setVerticalAngles(vertical_angles);
        parser.get<PARSER_04_INDEX>().setVerticalAngles(vertical_angles);
      }
    }
  }
  catch (boost::property_tree::ptree_error e)
  {
    std::cerr << "caught ptree error: " << e.what() << std::endl;
    return -1;
  }

  catch (std::exception e)
  {
    std::cerr << "caught exception in http client read: " << e.what() << std::endl;
    return -1;
  }


  // store connections for cleaner shutdown
  std::vector<boost::signals2::connection> connections;

  // connect the packets from the client to the parser
  connections.push_back(client.connect([&parser](const ClientType::ResultType& pc){ parser.slot(pc); }));
  
  // if we're doing automatic calibration or if we're setting the calibration,
  // include the calibrator in the chain
  if (perform_encoder_calibration)
  {
    // connect the parser to the calibrator
    connections.push_back(parser.connect([&calibrator](const ParserModuleType::ResultType& pc){ calibrator.slot(pc); }));
    // connect the calibrator to the converter
    connections.push_back(calibrator.connect([&converter](const CalibrationType::ResultType& pc){ converter.slot(pc); }));
  }
  else
  {
    // connect the parser to the converter
    connections.push_back(parser.connect([&converter](const ParserModuleType::ResultType& pc){ converter.slot(pc); }));
  }

  ////////////////////////////////////////////
  /// connect application specific logic here to consume the point cloud
  ////////////////////////////////////////////
  // connect the converter to the visualizer
  connections.push_back(converter.connect([&visualizer](const ConverterType::ResultType& pc){ visualizer.slot(pc); }));

  // run the client in a separate thread
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

  ////////////////////////////////////////
  /// put application specific logic here
  ////////////////////////////////////////
  // start visualizer (blocks until stopped)
  visualizer.run();

  // clean up
  client.stop();
  connections.clear();
  client_thread.join();

  return (0);
}
