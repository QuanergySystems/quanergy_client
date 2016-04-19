#include <cmath>
#include <gtest/gtest.h>
#include <quanergy/modules/encoder_angle_calibration.h>

namespace quanergy
{
  namespace test
  {
    class TestEncoderCalibration : public ::testing::Test
    {
    public:

      TestEncoderCalibration()
      {
      }

      virtual ~TestEncoderCalibration()
      {
      }

      virtual void SetUp()
      {

      }

      virtual void TearDown()
      {
      }

      quanergy::calibration::EncoderAngleCalibration encoder_calibration_;

    };

    TEST_F(TestEncoderCalibration, Test_fitLine)
    {
      // create a signal of encoder angles going from -pi to pi
      std::vector<double> encoder_angles;
      auto rads_per_encoder = 0.005;
      auto amplitude = 0.02;
      auto phase = -2.0;
      
      for (double angle = -M_PI; angle < M_PI; angle += rads_per_encoder)
      {
        encoder_angles.push_back(angle + (amplitude * std::sin(angle + phase)));
      }
      auto encoder_period = encoder_angles.size();

      auto slope = (2 * M_PI) / static_cast<double>(encoder_period);
      auto calculated_slope = encoder_calibration_.fitLine(encoder_angles);

      ASSERT_NEAR(slope, calculated_slope, 0.0001);
    }

    TEST_F(TestEncoderCalibration, Test_findSinusoidParameters)
    {
      // Generate observed error in encoder angles
      const double amplitude = 0.00432119;
      double phase_offset = -2000;// units: encoder counts

      std::vector<double> sinusoid;
      const double motor_speed_rps = 10.;
      const double sample_frequency = 54000.;
      const int encoder_period = (sample_frequency / motor_speed_rps);
      double phase_offset_rads = 2 * M_PI * phase_offset / encoder_period;

      // create a sinusoid
      for (int i = 0; i < encoder_period; i++)
      {
        sinusoid.push_back(amplitude * std::sin(((2 * M_PI / encoder_period) * i) + phase_offset_rads));
      }

      auto sine_params = encoder_calibration_.findSinusoidParameters(sinusoid);

      ASSERT_NEAR(sine_params.first, amplitude, 0.0001);
      ASSERT_NEAR(sine_params.second, phase_offset_rads, 0.0001);
    }

  }/** end test namespace */
}/** end quanergy namespace */

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

