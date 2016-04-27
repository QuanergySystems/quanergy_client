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

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

#include <quanergy/modules/encoder_angle_calibration.h>

#include <Eigen/Dense>

namespace quanergy
{
  namespace calibration
  {
    const double EncoderAngleCalibration::FIRING_RATE = 53828.;

    /** Once the motor has reached stead-state, the number of encoder counts per
     * revolution should be roughly the firing rate divided by the frame rate.
     * This number is how many counts the current revolution can be within the
     * theoretical steady-state number of encoder counts. */
    const int EncoderAngleCalibration::ENCODER_COUNT_TOLERANCE = 200;

    // I choose this value by looking at multiple segments of revolutions from
    // -pi to pi and found all endpoints were within this value of -pi and pi.
    const double EncoderAngleCalibration::PI_TOLERANCE = 0.01;

    /* Number of encoder counts to use when smoothing error signal */
    const int EncoderAngleCalibration::MOV_AVG_PERIOD = 300;

    /** This is the criteria for phase converging without outliers. If the
     * number of consecutive trials where the phase difference does not exceed
     * this number is above the total number of calibration trials, the
     * calibration is complete. */
    const double EncoderAngleCalibration::PHASE_CONVERGENCE_THRESHOLD = 0.1;

    /** Theshold dictating whether the encoder offset error requires
     * applying a calibration. If the amplitude is below this value, it is
     * likely that the sinusoidal error is insignificant. Calculating the offset
     * in this case is inconsistent since another source of noise dominates the
     * overall noise in the signal. This value was chosen to be roughly 5 times
     * the amplitude of this other noise */
    const double EncoderAngleCalibration::AMPLITUDE_THRESHOLD = 0.005;

    EncoderAngleCalibration::EncoderAngleCalibration(bool run_forever)
      : run_forever_(run_forever)
      , started_full_rev_(false)
      , calibration_complete_(false)
      , required_samples_(100)
      , num_valid_samples_(0)
      , first_run_(true)
      , calibration_count_(0)
    {
      unsigned int num_threads = std::thread::hardware_concurrency();
      for (int i = 0; i < num_threads; i++)
        futures_.push_back(std::async(
            std::launch::async, &EncoderAngleCalibration::processAngles, this));
    }

    EncoderAngleCalibration::~EncoderAngleCalibration()
    {
      calibration_complete_ = true;
      nonempty_condition_.notify_all();

      for (auto& future : futures_)
      {
        future.get();
      }
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

    void EncoderAngleCalibration::setTimeout(int timeout)
    {
      timeout_ = timeout;
    }

    void EncoderAngleCalibration::setRequiredNumSamples(double num_samples)
    {
      required_samples_ = num_samples;
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

      // if this is the first time to slot is called, we want to record the time
      // we started calibrating. If the timeout has elapsed, we'll want to throw
      // an exception.
      if (!run_forever_)
      {
        if (false == started_calibration_)
        {
          started_calibration_ = true;
          time_started_ = std::chrono::system_clock::now();
        }
        else
        {
          if (std::chrono::system_clock::now() - time_started_ >
              std::chrono::seconds(timeout_))
          {
            throw std::runtime_error(
                "Encoder calibration timed out."); 
          }
        }
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

              // push to queue which will be consumed by processAngles
              {
                std::lock_guard<decltype(queue_mutex_)> lock(queue_mutex_);
                period_queue_.push(encoder_angles);
                nonempty_condition_.notify_one();
              }

              hvdir_pts_.clear();
              started_full_rev_ = false;

              return;
            }

            hvdir_pts_.clear();
            started_full_rev_ = false;
            
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

    void EncoderAngleCalibration::processAngles()
    {
      while (!calibration_complete_ && num_valid_samples_ < required_samples_)
      {
        AngleContainer encoder_angles;
        // pop a vector of encoder angles off the queue
        {
          std::unique_lock<decltype(queue_mutex_)> lock(queue_mutex_);
          while (!calibration_complete_ && period_queue_.empty())
          {
            nonempty_condition_.wait(
                lock, [this]()
                {
                  return (!period_queue_.empty() || calibration_complete_);
                });
          }

          if (calibration_complete_)
            return;

          encoder_angles = period_queue_.front();
          period_queue_.pop();
        }
        
        // if we've finished collecting samples after being blocked, return
        if (calibration_complete_ || num_valid_samples_ > required_samples_)
          return;
        
        // calculate the amplitude and phase and display to user
        auto sine_parameters = calculate(encoder_angles, true);

        // if we've finished collecting samples after being blocked, return
        if (calibration_complete_ || num_valid_samples_ > required_samples_)
          return;

        if (sine_parameters.first < AMPLITUDE_THRESHOLD)
        {
          std::cout << "Correction amplitude of " << sine_parameters.first
                    << " is below threshold of " << AMPLITUDE_THRESHOLD
                    << ". No calibration will be applied." << std::endl;

          calibration_complete_ = true;
          return;
        }

        // display all results if we're running forever so we can analyze the
        // results. We won't be averaging in this mode so return immediately
        if (run_forever_)
        {
          if (first_run_)
          {
            std::cout << "AMPLITUDE(rads), PHASE(rads)" << std::endl;
            first_run_ = false;
          }

          std::stringstream output;
          output << sine_parameters.first << "," << sine_parameters.second
                 << std::endl;

          std::cout << output.str();
          return;
        }

        // At this point we're not running forever. We will collect valid
        // calibrations until enough have been collected and then average the
        // results and use those values as the calibration to apply to outgoing
        // points.

        // this will lock the container mutex until the end of the while loop
        // we want exclusive access for the rest of the loop
        std::lock_guard<decltype(container_mutex_)> lock(container_mutex_);
        
        // if we've finished collecting samples after being blocked, return
        if (calibration_complete_ || num_valid_samples_ > required_samples_)
          return;

        if (amplitude_values_.empty() && phase_values_.empty())
        {
          amplitude_values_.push_back(sine_parameters.first);
          phase_values_.push_back(sine_parameters.second);
          return;
        }

        if (std::fabs(sine_parameters.second - phase_values_.back()) < PHASE_CONVERGENCE_THRESHOLD)
        {
          amplitude_values_.push_back(sine_parameters.first);
          phase_values_.push_back(sine_parameters.second);

          num_valid_samples_++;

          if (num_valid_samples_ > required_samples_)
          {
            namespace ba = boost::accumulators;
            // average and report to user
            ba::accumulator_set<double, ba::stats<ba::tag::mean, ba::tag::variance>> amplitude_acc;
            ba::accumulator_set<double, ba::stats<ba::tag::mean, ba::tag::variance>> phase_acc;

            for (const auto& value : amplitude_values_)
              amplitude_acc(value);

            for (const auto& value : phase_values_)
              phase_acc(value);

            amplitude_ = ba::mean(amplitude_acc);
            phase_ = ba::mean(phase_acc);

            std::cout << "Calibration complete." << std::endl
              << "  amplitude : " << amplitude_ << std::endl
              << "  phase     : " << phase_ << std::endl;

            calibration_complete_ = true;
          }
        }
        else
        {
          // we've calculated a phase value which is outside the window of
          // convergence. Clear the amplitude and phase containers and start
          // again.
          amplitude_values_.clear();
          phase_values_.clear();
          num_valid_samples_ = 0;

          // add the current values to the containers
          amplitude_values_.push_back(sine_parameters.first);
          phase_values_.push_back(sine_parameters.second);
        }

      } // end of while(num_valid_samples < required_samples_)

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
      // if you divide the firing rate by the frame rate you get the number of
      // points per rev. Convert to radians
      auto rads_per_count = 2 * M_PI / (FIRING_RATE / frame_rate_);
      
      for (int i = 1; i < hvdir_pts_.size(); i++)
      {
        // TCP packets are in points chunks of 50. If we see a delta angle of
        // more than 5 encoder counts, discard as dropped packet.
        if (std::abs(hvdir_pts_[i].h - hvdir_pts_[i-1].h) > 5 * rads_per_count)
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
      std::vector<double> model;
      std::vector<double> sinusoid;

      auto calc_sinusoid =
          [&sinusoid, &encoder_angles, &model](double slope, double y_intercept)
      {
        model.clear();
        sinusoid.clear();
        for (int count = 0; count < encoder_angles.size(); count++)
        {
          model.push_back(slope * count + y_intercept);
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

      auto smoothed_sinusoid = movingAvgFilter(sinusoid, MOV_AVG_PERIOD);

      if (run_forever_)
      {
        std::stringstream encoder_filename;
        std::stringstream model_filename;
        std::stringstream error_filename;
        std::stringstream smoothed_filename;

        {
          std::lock_guard<decltype(file_mutex_)> lock(file_mutex_);

          encoder_filename << "encoder" << calibration_count_ << ".csv";
          model_filename << "model" << calibration_count_ << ".csv";
          error_filename << "error" << calibration_count_ << ".csv";
          smoothed_filename << "smoothed" << calibration_count_ << ".csv";

          calibration_count_++;
        }

        std::ofstream encoder_output{encoder_filename.str()};
        std::ofstream model_output{model_filename.str()};
        std::ofstream error_output{error_filename.str()};
        std::ofstream smoothed_output{smoothed_filename.str()};

        for (int i = 0; i < encoder_angles.size(); i++)
        {
          encoder_output << encoder_angles[i] << std::endl;
          model_output << model[i] << std::endl;
          error_output << sinusoid[i] << std::endl;
          smoothed_output << smoothed_sinusoid[i] << std::endl;
        }

        encoder_output.close();
        model_output.close();
        error_output.close();
        smoothed_output.close();
      }

      return (findSinusoidParameters(smoothed_sinusoid));
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

    EncoderAngleCalibration::AngleContainer
    EncoderAngleCalibration::movingAvgFilter(
        const AngleContainer& encoder_angles, int period)
    {
      namespace ba = boost::accumulators;

      auto half_period = static_cast<int>(period / 2);

      // assert size of container is greater than moving average period
      
      AngleContainer filtered_angles;
      for (auto signal_iter = encoder_angles.begin(); signal_iter != encoder_angles.end(); ++signal_iter)
      {
        // make sure we don't iterate over the endpoints. If we're at an
        // endpoint we'll just take an average up until the endpoint. This means
        // our filtered signal will be the same size as our input.
        auto beg_iter = std::distance(encoder_angles.begin(), signal_iter) < half_period
                            ? encoder_angles.begin()
                            : signal_iter - half_period;

        auto end_iter = std::distance(signal_iter, encoder_angles.end()) < half_period
                            ? encoder_angles.end()
                            : signal_iter + half_period;

        ba::accumulator_set<double, ba::stats<ba::tag::mean>> avg_acc;
        for (auto& avg_iter = beg_iter; avg_iter < end_iter; ++avg_iter)
          avg_acc(*avg_iter);

        filtered_angles.push_back(ba::mean(avg_acc));
      }

      return filtered_angles;
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

