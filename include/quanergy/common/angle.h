/****************************************************************
 **                                                            **
 **  Copyright(C) 2016 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#ifndef ANGLE_H_VOLOEZB8
#define ANGLE_H_VOLOEZB8

#include <vector>
#include <numeric>
#include <cmath>

namespace quanergy
{
  namespace common
  {
    /** 
     * @brief Calculates the difference between two angles using the
     * dot-product. This function will always return the absolute angle between
     * two angles.
     * 
     * @param[in] a First angle
     * @param[in] b Second angle
     * 
     * @return Difference between angles between 0 and pi.
     */
    template <typename Scalar>
    Scalar angleDiff(Scalar a, Scalar b)
    {
      return (std::acos((std::cos(a) * std::cos(b)) +
                        (std::sin(a) * std::sin(b))));
    }

    /** 
     * @brief Class to average angles
     */
    template<typename Scalar>
    class AngleAverager
    {
      public:
        /** 
         * @brief Add angle to accumulated set to be averaged
         * 
         * @param[in] angle Angle
         */
        void accumulate(Scalar angle)
        {
          x_values.push_back(std::cos(angle));
          y_values.push_back(std::sin(angle));
        }

        /** 
         * @brief Clear accumulation
         */
        void clear()
        {
          x_values.clear();
          y_values.clear();
        }

        bool empty() const
        {
          return (x_values.size() == 0);
        }

        /** 
         * @brief Get average angle. Implementation converts angles passed by
         * accumulate() into cartesian coordinates on the unit circle. X and Y
         * values for coordinate are averaged and the arctangent of these sums
         * is returned as the average. If the sums of the X and Y values on the
         * unit circle *both* equal zero, zero is returned as the average.
         * 
         * @return Average angle.
         */
        Scalar avg() const
        {
          Scalar sum_x = 0., sum_y = 0.;
          for (int i = 0; i < x_values.size(); i++)
          {
            sum_x += x_values[i];
            sum_y += y_values[i];
          }

          return (std::atan2(sum_y, sum_x));
        }

      private:
        std::vector<Scalar> x_values;
        std::vector<Scalar> y_values;
    };
  } // namespace common
} // namespace quanergy

#endif /* end of include guard: ANGLE_H_VOLOEZB8 */

