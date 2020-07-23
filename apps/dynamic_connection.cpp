/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

// control flow management
#include <mutex>
#include <condition_variable>

// console parser
#include <boost/program_options.hpp>

// TCP client for sensor
#include <quanergy/client/sensor_client.h>

// sensor pipeline
#include <quanergy/pipelines/sensor_pipeline.h>

int main(int argc, char** argv)
{
  namespace po = boost::program_options;

  po::options_description description("Quanergy Client Dynamic Connection");
  const po::positional_options_description p; // empty positional options

  quanergy::pipeline::SensorPipelineSettings pipeline_settings;
  std::string return_string;
  std::vector<float> correct_params;

  // port
  std::string port = "4141";

  description.add_options()
    ("help,h", "Display this help message.")
    ("settings-file,s", po::value<std::string>(),
      "Settings file. Setting file values override defaults and command line arguments override the settings file.")
    ("host", po::value<std::string>(&pipeline_settings.host),
      "Host name or IP of the sensor.")
    ("frame,f", po::value<std::string>(&pipeline_settings.frame)->
      default_value(pipeline_settings.frame),
      "Frame name inserted in the point cloud.")
    ("return,r", po::value<std::string>(&return_string),
      "Return selection (M-series only) - "
      "Options are 0, 1, 2, or all. For 3 return packets, 'all' creates an unorganized point cloud. "
      "For single return, explicitly setting a value produces an error if the selection doesn't match the packet.")
    ("calibrate", po::bool_switch(&pipeline_settings.calibrate),
      "Flag indicating encoder calibration should be performed and applied to outgoing points; M-series only.")
    ("frame-rate", po::value<double>(&pipeline_settings.frame_rate)->
      default_value(pipeline_settings.frame_rate),
      "Frame rate used when peforming encoder calibration; M-series only.")
    ("manual-correct", po::value<std::vector<float>>(&correct_params)->multitoken()->value_name("amplitude phase"),
      "Correct encoder error with user defined values. Both amplitude and phase are in radians; M-series only.")
    ("min-distance", po::value<float>(&pipeline_settings.min_distance)->
      default_value(pipeline_settings.min_distance),
      "minimum distance (inclusive) for distance filtering.")
    ("max-distance", po::value<float>(&pipeline_settings.max_distance)->
      default_value(pipeline_settings.max_distance),
      "maximum distance (inclusive) for distance filtering.")
    ("min-cloud-size", po::value<std::int32_t>(&pipeline_settings.min_cloud_size)->
      default_value(pipeline_settings.min_cloud_size),
      "minimum cloud size; produces an error and ignores clouds smaller than this.")
    ("max-cloud-size", po::value<std::int32_t>(&pipeline_settings.max_cloud_size)->
      default_value(pipeline_settings.max_cloud_size),
      "maximum cloud size; produces an error and ignores clouds larger than this.");

  try
  {
    // load the command line options into the variables map
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(description).positional(p).run(), vm);

    if (vm.count("help"))
    {
      std::cout << description << std::endl;
      return 0;
    }

    // if there is a settings file, load that before notifying (which fills the variables)
    if (vm.count("settings-file"))
    {
      std::string settings_file = vm["settings-file"].as<std::string>();
      quanergy::pipeline::SettingsFileLoader file_loader;
      file_loader.loadXML(settings_file);
      pipeline_settings.load(file_loader);
    }

    // notify; this stores command line options in associated variables
    po::notify(vm);

    // let the user know if there is no host value
    if (pipeline_settings.host.empty())
    {
      std::cout << "No host provided" << std::endl;
      std::cout << description << std::endl;
      return -1;
    }

    // handle return selection
    if (!return_string.empty())
    {
      pipeline_settings.return_selection_set = true;
      pipeline_settings.return_selection = pipeline_settings.returnFromString(return_string);
    }

    // handle encoder correction parameters
    if (!correct_params.empty())
    {
      if (correct_params.size() == 2)
      {
        pipeline_settings.override_encoder_params = true;
        pipeline_settings.amplitude = correct_params[0];
        pipeline_settings.phase = correct_params[1];
      }
      else
      {
        std::cout << "Manual encoder correction expects exactly 2 paramters: amplitude and phase" << std::endl;
        std::cout << description << std::endl;
        return -1;
      }
    }
  }
  catch (po::error& e)
  {
    std::cout << "Boost Program Options Error: " << e.what() << std::endl << std::endl;
    std::cout << description << std::endl;
    return -1;
  }
  catch (std::exception& e)
  {
    std::cout << "Error: " << e.what() << std::endl;
    return -2;
  }


  // create client to get raw packets from the sensor
  quanergy::client::SensorClient client(pipeline_settings.host, port, 100);

  // create pipeline to produce point cloud from raw packets
  quanergy::pipeline::SensorPipeline pipeline(pipeline_settings);

  // store connections for cleaner shutdown
  std::vector<boost::signals2::connection> connections;

  ////////////////////////////////////////////
  /// if you'd like to parse the packets yourself, connect here
  ////////////////////////////////////////////
  // connect the packets from the client to the sensor pipeline
  connections.push_back(client.connect(
      [&pipeline](const std::shared_ptr<std::vector<char>>& packet){ pipeline.slot(packet); }
  ));
  
  ////////////////////////////////////////////
  /// connect application specific logic here to consume the point cloud
  ////////////////////////////////////////////
  // here we'll simply count the number of packets and output every 100
  unsigned int cloud_count = 0;
  connections.push_back(pipeline.connect(
      [&cloud_count](const boost::shared_ptr<pcl::PointCloud<quanergy::PointXYZIR>>& /*pc*/)
      { ++cloud_count; if(cloud_count % 100 == 0) std::cout << "clouds received: " << cloud_count << std::endl; }
  ));

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
    ////////////////////////////////////////
    /// put application specific logic here related to starting a stopping
    ////////////////////////////////////////
    // here we'll simply use a CLI
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

        if (pipeline_settings.calibrate)
        {
          pipeline.encoder_corrector.reset();
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
