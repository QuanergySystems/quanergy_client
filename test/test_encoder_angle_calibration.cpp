/****************************************************************
 **                                                            **
 **  Copyright(C) 2016 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <stdexcept>
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

    TEST_F(TestEncoderCalibration, Test_calculate)
    {
      // create a signal of encoder angles going from -pi to pi
      std::vector<double> encoder_angles;
      auto rads_per_encoder = 0.005;
      auto amplitude = 0.02;
      auto phase = -2.0;

      auto encoder_noise = std::bind(
          std::uniform_real_distribution<>{-0.002, 0.002},
          std::default_random_engine{});

      // construct encoder angles where motor is moving in clockwise direction
      for (double angle = M_PI; angle > -M_PI; angle -= rads_per_encoder)
      {
        encoder_angles.push_back(angle + (amplitude * std::sin(angle + phase)) + encoder_noise());
      }

      auto sine_parameters = encoder_calibration_.calculate(encoder_angles);

      ASSERT_NEAR(sine_parameters.first, amplitude, 0.005);
      // the phase calculation will vary due to the noise applied to the
      // signal so we give it a fairly large tolerance to pass the test. During
      // calibration, this value is averaged over 100 calibations.
      ASSERT_NEAR(sine_parameters.second, phase, 0.5);

      // construct encoder angles where motor is moving in counter-clockwise
      // direction
      encoder_angles.clear();
      for (double angle = -M_PI; angle < M_PI; angle += rads_per_encoder)
      {
        encoder_angles.push_back(angle + (amplitude * std::sin(angle + phase)));
      }

      sine_parameters = encoder_calibration_.calculate(encoder_angles);

      ASSERT_NEAR(sine_parameters.first, amplitude, 0.005);
      ASSERT_NEAR(sine_parameters.second, phase, 0.5);
    }

    TEST_F(TestEncoderCalibration, Test_calculateEmptyInput)
    {
      std::vector<double> encoder_angles;
      EXPECT_THROW(encoder_calibration_.calculate(encoder_angles),
                   std::runtime_error);
    }

    TEST_F(TestEncoderCalibration, Test_calculateConstantSignal)
    {
      std::vector<double> encoder_angles;
      for (int i = 0; i < 1000; i++)
        encoder_angles.push_back(0);

      EXPECT_THROW(encoder_calibration_.calculate(encoder_angles),
                   std::runtime_error);
    };

  }/** end test namespace */
}/** end quanergy namespace */

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

