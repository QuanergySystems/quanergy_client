/****************************************************************
 **                                                            **
 **  Copyright(C) 2020 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <quanergy/pipelines/sensor_pipeline_settings.h>

#include <boost/lexical_cast.hpp>

using namespace quanergy::pipeline;

int SensorPipelineSettings::returnFromString(const std::string& r)
{
  int ret;

  if (r == "all")
  {
    ret = quanergy::client::ALL_RETURNS;
  }
  // Verify argument contains only digits
  else if (!r.empty() && std::all_of(r.begin(), r.end(), ::isdigit))
  {
    ret = std::atoi(r.c_str());
    if (ret < 0 || ret >= quanergy::client::M_SERIES_NUM_RETURNS)
    {
      throw std::invalid_argument("Invalid return selection");
    }
  }
  else
  {
    throw std::invalid_argument("Invalid return selection");
  }

  return ret;
}

std::string SensorPipelineSettings::stringFromReturn(int r)
{
  std::string ret;

  if (r == quanergy::client::ALL_RETURNS)
  {
    ret = "all";
  }
  else if (r >= 0 && r < quanergy::client::M_SERIES_NUM_RETURNS)
  {
    ret = std::to_string(r);
  }
  else
  {
    throw std::invalid_argument("Invalid return selection");
  }

  return ret;
}

void SensorPipelineSettings::load(const SettingsFileLoader& settings)
{
  host = settings.get("Settings.host", host);

  frame = settings.get("Settings.frame", frame);

  auto r = settings.get_optional<std::string>("Settings.return");
  if (r && !r->empty())
  {
    return_selection_set = true;
    return_selection = returnFromString(*r);
  }

  min_distance = settings.get("Settings.DistanceFilter.min", min_distance);
  max_distance = settings.get("Settings.DistanceFilter.max", max_distance);

  calibrate = settings.get("Settings.EncoderCorrection.calibrate", calibrate);
  frame_rate = settings.get("Settings.EncoderCorrection.frameRate", frame_rate);
  override_encoder_params = settings.get("Settings.EncoderCorrection.override", override_encoder_params);
  amplitude = settings.get("Settings.EncoderCorrection.amplitude", amplitude);
  phase = settings.get("Settings.EncoderCorrection.phase", phase);

  min_cloud_size = settings.get("Settings.minCloudSize", min_cloud_size);
  max_cloud_size = settings.get("Settings.maxCloudSize", max_cloud_size);

  /// ring filter settings only relevant for M-series
  for (int i = 0; i < quanergy::client::M_SERIES_NUM_LASERS; i++)
  {
    const std::string num = boost::lexical_cast<std::string>(i);

    std::string range_param = std::string("Settings.RingFilter.range").append(num);
    ring_range[i] = settings.get(range_param, ring_range[i]);

    std::string intensity_param = std::string("Settings.RingFilter.intensity").append(num);
    ring_intensity[i] = settings.get(intensity_param, ring_intensity[i]);
  }

}
