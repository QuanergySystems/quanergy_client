/****************************************************************
 **                                                            **
 **  Copyright(C) 2016 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#ifndef ENCODER_ANGLE_CALIBRATION_H_
#define ENCODER_ANGLE_CALIBRATION_H_

#include <vector>
#include <iostream>
#include <future>
#include <mutex>
#include <atomic>
#include <queue>
#include <condition_variable>

#include <boost/signals2.hpp>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

#include <quanergy/common/point_hvdir.h>
#include <quanergy/common/pointcloud_types.h>
#include <quanergy/common/angle.h>

#ifdef _MSC_VER
  #define DLLEXPORT __declspec(dllexport)
  #define TYPENAME
#else
  #define DLLEXPORT
  #define TYPENAME typename
#endif

namespace quanergy
{
  namespace calibration
  {
    /**
     * @brief This class calculates the error in the encoder angles and returns
     * the amplitude and phase shift of the sine function modeling the error in
     * the encoder angles for the M8 Sensor. This class can also be provided
     * with the sine error parameters and apply the calibration to incoming
     * points.
     */
	  struct DLLEXPORT EncoderAngleCalibration
    {
      /** typedef for accumulator set */
      using AccumulatorSet = boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::mean, boost::accumulators::tag::variance>>;

      /** The type of the container that contains the encoder
       * angles */
      using AngleContainer = std::vector<double>;

      /** This type holds the parameters of the sine wave.
       * The amplitude is the first element of the pair,
       * the phase is the second element. */
      using SineParameters = std::pair<double, double>;

      /** This type holds the parameters of a 2d line */
      using LineParameters = std::pair<double, double>;

      /** 
       * @brief Short-hand for point types
       */
      using Ptr = std::shared_ptr<EncoderAngleCalibration>;

      /** 
       * @brief Result type for class
       */
      using ResultType = PointCloudHVDIRPtr;

      /** 
       * @brief Signal type
       */
      using Signal =  boost::signals2::signal<void (const ResultType&)>;

      /** The firing rate of the LiDAR, in Hz */
      static const double FIRING_RATE;

      /** Allowable tolerance within pi for an endpoint-angle in a revolution to
       * be considered near pi */
      static const double PI_TOLERANCE;

    public:

      /** 
       * @brief Constructor
       */
      EncoderAngleCalibration();

      /**
       * @brief Empty destructor
       */
      ~EncoderAngleCalibration();

      /**
       * @brief Resets the calibrator so it can be run again
       */
      void reset();

      /** 
       * @brief Adds subscriber to be called after this classes functionality is
       * done.
       * 
       * @param[in] subscriber Subscriber to be called.
       * 
       * @return connection between this class and subscriber
       */
      boost::signals2::connection connect(const TYPENAME Signal::slot_type& subscriber);

      /** 
       * @brief Slot to be connected as a subscriber to another process. If
       * calibration is not complete, this function will add the point cloud
       * argument to the cloud to be used for calibration. If calibration is
       * complete, this function will apply the calibration and call the next
       * subscriber.
       * 
       * @param[in] pc Point cloud to be processed.
       */
      void slot(PointCloudHVDIRPtr const & pc);

      /** 
       * @brief Sets this class to only calculate the error parameters and not
       * apply the calibration. This mode is for when the caller wants to look
       * at multiple calculations for the amplitude and phase.
       */
      void calibrateOnly();

      /**
       * @brief Sets number of valid calibrations to be collected before
       *averaging. Validity is determined by change in phase between current
       *sample and previous sample
       *
       * @param[in] num_cals Number of calibrations
       */
      void setRequiredNumSamples(double num_cals);

      /** 
       * @brief Function to manually set calibration parameters. Calling this
       * function will disable the automatic calibration and subsequent calls to
       * slot will apply the calibration and call the subscriber.
       * 
       * @param[in] amplitude Amplitude of sinusoid error
       * @param[in] phase Phase of sinusoidal error
       */
      void setParams(double amplitude, double phase);

      /** 
       * @brief Function to set frame rate. Value is in frames per second. If
       * not set, the default is 10.
       * 
       * @param[in] frame_rate frame rate.
       */
      void setFrameRate(double frame_rate);

      /**
       * @brief Sets timeout for calculating the calibration. If the timeout
       * expires this class will throw an exception.
       *
       * @param[in] timeout Timeout.
       */
      template<typename Rep, typename Period>
      void setTimeout(const std::chrono::duration<Rep,Period>& timeout)
      {
        timeout_ = std::chrono::duration_cast<std::chrono::seconds>(timeout);
      }

      /**
       * @brief Function to calculate the sinusoidal error of the horizontal
       * encoder values. This function is called once a full revolution of the
       * encoder is captured.
       *
       * @param[in] encoder_angles Encoder angles
       *
       * @return Tuple with first element as the amplitude of the sinusoid and
       * second element as the phase offset of the sinusoid.
       */
      SineParameters calculate(const std::vector<double>& encoder_angles);

      /** 
       * @brief Sets the number of motor encoder counts within the theoretical
       * steady-state expected number when determining if the motor has reached
       * steady-state.
       * 
       * @param[in] tolerance Number of encoder counts.
       */
      inline void setEncoderCountTolerance(int tolerance)
      {
        encoder_count_tolerance_ = tolerance;
      }

      /** 
       * @brief Sets the number of encoder counts used to smooth out the error
       * signal before modeling the sinusoid.
       * 
       * @param[in] period Period in encoder counts.
       */
      inline void setMovingAveragePeriod(int period)
      {
        moving_average_period_counts_ = period;
      }

      /** 
       * @brief Sets phase convergence threshold. This class continually models
       * sinusoid curves to the error until the difference between phase
       * calculations is below this value.
       * 
       * @param[in] threshold Phase convergence threshold, in radians.
       */
      inline void setPhaseConvergenceThreshold(double threshold)
      {
        phase_convergence_threshold_ = threshold;
      }

      /** 
       * @brief Sets the amplitude threshold. If a timeout occurs waiting for
       * encoder calibration to occur, the average amplitude calculated will be
       * checked against this value. If the average amplitude is less than the
       * amplitude_threshold_, the user will be notified that the encoder
       * calibration cannot be accurately applied for such low error.
       * 
       * @param[in] threshold Amplitude threshold, in radians
       */
      inline void setAmplitudeThreshold(double threshold)
      {
        amplitude_threshold_ = threshold;
      }

    private:
      /**
       * @brief Function to create line representing the expected encoder
       * values. Unwrapped encoder values should be linear with respect to 
       * time-index. This line is used to determine the error.
       *
       * @param[in] encoder_angles Angles to fit line to
       * @returns slope of the line
       */
      static double fitLine(const AngleContainer& encoder_angles);

      /**
       * @brief Function to determine sinusoidal parameters from a signal. Once
       * we've created a sinusoid of the encoder error, we use this function to
       * determine the parameters of the sinusoid.
       *
       * @param[in] sinusoid_values Sinusoid signal
       * @param[in] clockwise Whether the motor is spinning clockwise.
       * If false, motor is spinning counter-clockwise.
       *
       * @return tuple where first element is amplitude of sinusoid and second
       * elemement is phase offset of sinusoid.
       */
      static SineParameters findSinusoidParameters(const AngleContainer& sine_signal, bool clockwise);

      /**
       * @brief Translate angle values so they are *not* contained within -pi
       *and
       * pi.
       *
       * @param[in] encoder_angles Encoder angles to be unwrapped.
       * @returns Unwrapped encoder angles
       */
      static AngleContainer unwrapEncoderAngles(const AngleContainer& encoder_angles);

      /** 
       * @brief Applies moving average in place
       * 
       * @param[inout] encoder_angles Signal to be filtered.
       * @param[in] period Period to apply averaging over, in container elements
       *
       */
      static AngleContainer movingAvgFilter(const AngleContainer& encoder_angles, int period);

      /** 
       * @brief Function to check if hvdir_pts_ has any gaps due to dropped
       * packets
       * 
       * @return True if HVDIR points are complelte with no dropped, false
       * otherwise. 
       */
      bool checkComplete() const;

      /** 
       * @brief Function to process encoder angles in thread pool
       */
      void processAngles();

      /** 
       * @brief Applies calibration. Applies calibration in place.
       * 
       * @param[in] cloud_ptr Point cloud calibration will be applied to.
       */
      void applyCalibration(PointCloudHVDIRPtr const & cloud_ptr) const;

      /** Once the motor has reached stead-state, the number of encoder counts per
       * revolution should be roughly the firing rate divided by the frame rate.
       * This number is how many counts the current revolution can be within the
       * theoretical steady-state number of encoder counts. */
      int encoder_count_tolerance_ = 200;
      
      /* Number of encoder counts to use when smoothing error signal, in encoder
       * ticks */
      int moving_average_period_counts_ = 300;

      /** This class continually fits a sinusoid to the error between the
       * expected horizontal angles and the horizontal angles received from the
       * M8. We calculate this sinusoid model until the phase values converge to
       * within this value. This value is in radians. */
      double phase_convergence_threshold_ = 0.1;

      /** Signal object to notify next slot */
      Signal signal_;

      /** Container for encoder angle values */
      AngleContainer encoder_angles_;

      /** thread pool futures */
      std::vector<std::future<void>> futures_;

      /** Mutex for period_queue_ */
      mutable std::mutex queue_mutex_;

      /** Condition variable for waiting on empty queue_mutex_ */
      std::condition_variable nonempty_condition_;

      /** queue for encoder angles vectors to be stored. Each element in the
       * queue contains a vector of encoder angles for a full period */
      std::queue<AngleContainer> period_queue_;

      /** Flag indicating calibration is complete. Used to withstand spurious
       * wakeups from condition variable */
      std::atomic<bool> calibration_complete_ {false};

      /** Calculated amplitude */
      double amplitude_ = 0.;

      /** Calculated phase (radians) */
      double phase_ = 0.;

      /** Frame rate of M8 sensor */
      double frame_rate_ = 10.;

      /** Flag indicating that we're outputting calibration results constantly
       * and never applying calibration. This mode is used when the user wants
       * to analyze the calibration results */
      bool run_forever_ = false;

      /** mutex around writing to file */
      mutable std::mutex file_mutex_;

      /** number of encoder calibrations to run before before averaging
       * amplitude and phase values and reporting these to user */
      std::atomic<int> required_samples_ {100};

      /** number of calibrations which have currently been processed. Used to
       * check against required_samples_ */
      std::atomic<int> num_valid_samples_ {0};

      /** mutex for containers holding amplitude_values_ and phase_values_ */
      mutable std::mutex container_mutex_;

      /** Accumulator to hold amplitude values. This is added to as valid calibration
       * samples are calculated and used to eventually calculate the average */
      AccumulatorSet amplitude_accumulator_;

      /** Special averaging container to calculate average of phase angles and
       * handle wrapping. */
      quanergy::common::AngleAverager<double> phase_averager_;

      /** Last phase angle processed by thread pool */
      double last_phase_ = 0.;

      /** Flag indicating calibration has started */
      bool started_calibration_ = false;

      /** time when calibration started */
      std::chrono::time_point<std::chrono::system_clock> time_started_;

      /** Timeout to collect enough calibrations. Defaulted is 60 seconds */
      std::chrono::seconds timeout_ = std::chrono::seconds(60);

      /** flag indicating whether or not the first calibration has been
       * performed. This flag gets set to false after the first run */
      std::atomic<bool> first_run_ {true};

      /** Number of calibrations which have occurred */
      std::atomic<int> calibration_count_ {0};

      /** Struct to hold statitistics on encoder calibration to report to user
       * if failed calibration occurs. */
      struct StatsType
      {
        int complete_frames = 0;
        int num_incomplete_frames = 0;
        int num_divergent_phase_values = 0;
      } stats_;

      /** Minimum amplitude (in rads) where a sinusoid error model is
       * appropriate for encoder error. If a timeout occurs when waiting for
       * the phase to converge, the average amplitude will be checked against
       * this value to determine if a calibration should be applied to the
       * sensor. This default value was determined experimentally. */
      double amplitude_threshold_ = 0.006; // rads
    };

  } /* calibration */
} /* quanergy */

#endif /* ENCODER_ANGLE_CALIBRATION_H_ */
