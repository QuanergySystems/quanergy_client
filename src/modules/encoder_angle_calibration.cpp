/****************************************************************
 **                                                            **
 **  Copyright(C) 2016 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <algorithm>
#include <map>
#include <cmath>
#include <chrono>
#include <fstream>
#include <csignal>

#include <quanergy/modules/encoder_angle_calibration.h>
#include <Eigen/Dense>

namespace quanergy
{
  namespace calibration
  {
    const double EncoderAngleCalibration::FIRING_RATE = 53828.;

    // This value was determined by running many calibration trials and noting
    // the minimum acceptable angles between firings. This threshold provides a
    // way to determie if packets were dropped when analyzing if a revolution is
    // appropriate for calibration.
    const double EncoderAngleCalibration::RADIANS_PER_ENCODER_COUNT = 0.005;

    /** Once the motor has reached stead-state, the number of encoder counts per
     * revolution should be roughly the firing rate divided by the frame rate.
     * This number is how many counts the current revolution can be within the
     * theoretical steady-state number of encoder counts. */
    const int EncoderAngleCalibration::ENCODER_COUNT_TOLERANCE = 200;

    // I choose this value by looking at multiple segments of revolutions from
    // -pi to pi and found all endpoints were within this value of -pi and pi.
    const double EncoderAngleCalibration::PI_TOLERANCE = 0.01;

    EncoderAngleCalibration::EncoderAngleCalibration()
    {
      started_full_rev_ = false;
      calibration_complete_ = false;
    }

    void EncoderAngleCalibration::setFrameRate(double frame_rate)
    {
      frame_rate_ = frame_rate;
    }

    boost::signals2::connection EncoderAngleCalibration::connect(
        const TYPENAME Signal::slot_type& subscriber)
    {
      return signal_.connect(subscriber);
    }

    EncoderAngleCalibration::~EncoderAngleCalibration()
    {
      if (processing_thread_.joinable())
        processing_thread_.join();
    }


    void EncoderAngleCalibration::slot(PointCloudHVDIRPtr const & cloud_ptr)
    {
      if (!cloud_ptr)
        return;

      if (calibration_complete_)
      {
        applyCalibration(cloud_ptr);
        return;
      }

      // Add the points to a point cloud. Do this until we have enough points to
      // to check for a complete revolution
      for (const auto& pt : *cloud_ptr)
      {
        // check fo discontinuity. If found, attempt calibration
        if (!hvdir_pts_.empty() && std::abs(hvdir_pts_.back().h - pt.h) > M_PI)
        {
          if (!started_full_rev_)
          {
            hvdir_pts_.clear();
            started_full_rev_ = true;
          }
          else
          {
            // we're at the end of a full-revolution. It's time to attempt
            // calibration
            if (checkComplete())
            {
              AngleContainer encoder_angles;
              for (const auto& pt : hvdir_pts_)
              {
                encoder_angles.push_back( pt.h );
              }

              processing_thread_ =
                  std::thread(std::bind(&EncoderAngleCalibration::processAngles,
                                        this, encoder_angles));
              processing_thread_.detach();

              hvdir_pts_.clear();
              started_full_rev_ = false;

              return;
            }

            hvdir_pts_.clear();
            started_full_rev_ = false;
            
            break;
          }
        }

        hvdir_pts_.push_back(pt);
      }
    }

    void EncoderAngleCalibration::setParams(double amplitude, double phase)
    {
      amplitude_ = amplitude;
      phase_ = phase;

      calibration_complete_ = true;
    }

    void EncoderAngleCalibration::applyCalibration(PointCloudHVDIRPtr const & cloud_ptr)
    {
      if (!cloud_ptr)
        return;

      // return immediately if there are no slots
      if (signal_.num_slots() == 0)
        return;

      PointCloudHVDIR & cloud = *cloud_ptr;

      for (auto& point : cloud)
      {
        // corrects in place, saves copying other values
        point.h = point.h - (amplitude_ * sin(point.h + phase_));
      }

      signal_(cloud_ptr);

    }

    void EncoderAngleCalibration::processAngles(AngleContainer encoder_angles)
    {
      auto sine_parameters = calculate(encoder_angles, true);

      amplitude_ = sine_parameters.first;
      phase_ = sine_parameters.second;

      std::cout << "amplitude: " << sine_parameters.first
                << " | phase: " << sine_parameters.second
                << " | encoder period: " << encoder_angles.size() << std::endl;

      calibration_complete_ = true;
    }

    bool EncoderAngleCalibration::checkComplete()
    {
      auto min = std::min(hvdir_pts_.front().h, hvdir_pts_.back().h);
      auto max = std::max(hvdir_pts_.front().h, hvdir_pts_.back().h);

      // check to make sure the front and back horizontal angle elements are
      // near the expected endpoints (-pi and pi)
      if (max < (M_PI - PI_TOLERANCE) || min > (-M_PI + PI_TOLERANCE))
        return (false);

      // check that the encoder period is within the steady-state range
      if ( hvdir_pts_.size() > ((FIRING_RATE / frame_rate_) + ENCODER_COUNT_TOLERANCE) ||
          hvdir_pts_.size() < ((FIRING_RATE / frame_rate_) - ENCODER_COUNT_TOLERANCE) )
        return (false);

      // iterate over hvdir_pts_ and check to make sure each point is within
      // RADIANS_PER_ENCODER_COUNT
      
      for (int i = 1; i < hvdir_pts_.size(); i++)
      {
        if (std::abs(hvdir_pts_[i].h - hvdir_pts_[i-1].h) > RADIANS_PER_ENCODER_COUNT)
          return (false);
      }

      return (true);
    }

    EncoderAngleCalibration::SineParameters EncoderAngleCalibration::calculate(
        const AngleContainer& encoder_angles,
        bool debugging)
    {
      debugging_ = debugging;

      auto slope = fitLine(encoder_angles);

      // calculate an initial error sinusoid from the line. This will tell us
      // how much our line needs to be shifted such that the sinusoid should be
      // vertically shifted. We will shift the line by this value and
      // recalculate our sinusoid
      std::vector<double> sinusoid;

      auto calc_sinusoid =
          [&sinusoid, &encoder_angles](double slope, double y_intercept)
      {
        sinusoid.clear();
        for (int count = 0; count < encoder_angles.size(); count++)
        {
          // no y-intercept this time
          sinusoid.push_back(encoder_angles[count] - slope * count - y_intercept);
        }
      };

      calc_sinusoid(slope, 0.);

      // determine the sinusoid vertical translation from the x-axis
      auto vertical_offset = (*std::max_element(sinusoid.begin(), sinusoid.end()) +
                              *std::min_element(sinusoid.begin(), sinusoid.end())) /
                             2;

      calc_sinusoid(slope, vertical_offset);

      return (findSinusoidParameters(sinusoid));
    }

      EncoderAngleCalibration::AngleContainer
      EncoderAngleCalibration::unwrapEncoderAngles(
          const AngleContainer& encoder_angles)
    {
      // create a new vector with the new contents
      AngleContainer unwrapped_encoder_values;

      // we set a threshold to identify the 0 encoder value when we're iterating
      // over the vector. We want to identify the zero-crossing but not identify
      // a
      // discontinuity. If the difference in angle is greater than this
      // threshold,
      // we're at a discontinuity
      const double discontinuity_threshold = M_PI;

      double last_encoder_angle = encoder_angles[0];
      double unwrap_addition = 0;

      // we want to catch the second zero crossing to stop
      int zero_crossing_count = 0;
      for (const auto& angle : encoder_angles)
      {
        // if we're going to positive to negative or negative to positive (zero
        // crossing) AND we're below the discontinuity threshold, it's a zero
        // crossing
        // We only care about the zero-crossing for the first value
        if (((last_encoder_angle < 0 && angle >= 0) ||
             (last_encoder_angle > 0 && angle <= 0)) &&
            std::abs(angle - last_encoder_angle) < discontinuity_threshold)
        {
          if (unwrapped_encoder_values.empty())
          {
            // start recording unwrapped angles
            unwrapped_encoder_values.push_back(angle);
            last_encoder_angle = angle;
            zero_crossing_count++;
            continue;
          }
          else
          {
            zero_crossing_count++;
          }
        }

        if (zero_crossing_count > 1) 
          break;

        if (unwrapped_encoder_values.empty())
        {
          last_encoder_angle = angle;
          continue;
        }

        if (std::abs(angle - last_encoder_angle) > discontinuity_threshold)
        {
          // we're at a discontinuity
          if (angle > last_encoder_angle)
          {
            // we went from -Pi to Pi, so we have a negative slope. Subtract
            // 2*Pi
            // from unwrap addition
            unwrap_addition -= 2 * M_PI;
          }
          else
          {
            // the opposite of above
            unwrap_addition += 2 * M_PI;
          }
        }

        unwrapped_encoder_values.push_back(angle + unwrap_addition);
        last_encoder_angle = angle;
      }

      return (unwrapped_encoder_values);
    }

    double EncoderAngleCalibration::fitLine(const AngleContainer& encoder_angles)
    {
      return (encoder_angles.back() - encoder_angles.front()) / encoder_angles.size();
    }

    EncoderAngleCalibration::SineParameters
    EncoderAngleCalibration::findSinusoidParameters(
        const AngleContainer& sine_signal)
    {

      // First, compute the amplitude. It will be the maximum value of the
      // signal:
      auto amplitude =
          *std::max_element(sine_signal.begin(), sine_signal.end());

      auto it_max = std::max_element(sine_signal.begin(), sine_signal.end());
      auto it_min = std::min_element(sine_signal.begin(), sine_signal.end());

      auto max_index = std::distance(sine_signal.begin(),it_max);
      auto min_index = std::distance(sine_signal.begin(),it_min);


      // Now, we will save the time shift of the sinusoid is the
      // time offset between the minimum and maximum peaks. This is
      // where the signal approximately crosses through zero.
      
      if (min_index == max_index)
        throw std::runtime_error("Peak detection found min and max peaks to be same value");
      
      double phase_index = 0;
      if (min_index < max_index)
        phase_index = (min_index + max_index) / 2;
      else
        // if min > max, add half a period
        phase_index = ((min_index + max_index) / 2) + (sine_signal.size() / 2);

      // From the phase index, compute the phase angle,
      // phi = 2*pi*f*t. The negative comes from the fact that
      // we search for the zero crossing to the right of the origin,
      // which indicates a shift to the right, given by a negative
      // phase:
      auto phase = std::fmod(2 * M_PI * phase_index / sine_signal.size(), 2 * M_PI);

      // we calculate the negative phase above, return the actual phase
      return std::make_pair(amplitude, -phase);
    }

  } /* calibration */
} /* quanergy */

