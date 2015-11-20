/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

/** \file point_xyzir.h
 *
 *  \brief Point Cloud Library point structures for cartesian data with intensity and ring number.
 *
 */

#ifndef QUANERGY_COMMON_POINT_XYZIR_H
#define QUANERGY_COMMON_POINT_XYZIR_H

#if 0
/// Using c++11 is broken with ROS Indigo before patch #484
/// (August 4th, 2014)
/// So not using std::uint16_t for now

#include <cstdint>
#else
#include <stdint.h>
#endif 

#include <limits>

#include <pcl/point_types.h>

#include <quanergy/common/point_xyz.h>

namespace quanergy
{
  /** Euclidean coordinate, including intensity and ring number. */
  struct PointXYZIR
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float               intensity;      ///< laser intensity reading
    uint16_t            ring;           ///< laser ring number

    PointXYZIR (const PointXYZIR& p)
      : x(p.x)
      , y(p.y)
      , z(p.z)
      , intensity(p.intensity)
      , ring(p.ring)
    {
      data[3] = 1.f;
    }

    PointXYZIR ()
      : x(0.0f)
      , y(0.0f)
      , z(0.0f)
      , intensity(0.0f)
      , ring(std::numeric_limits<uint16_t>::max())
    {
      data[3] = 1.f;
    }

    PointXYZIR (float _x, 
                float _y, 
                float _z, 
                float _intensity = 0.0f, 
                uint16_t _ring = std::numeric_limits<uint16_t>::max())
      : x(_x)
      , y(_y)
      , z(_z)
      , intensity(_intensity)
      , ring(_ring)
    {
      data[3] = 1.f;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

  PointXYZ operator+(PointXYZIR const &, float);
  PointXYZ operator-(PointXYZIR const &, float);
  PointXYZ operator*(PointXYZIR const &, float);
  PointXYZ operator/(PointXYZIR const &, float);
  PointXYZ operator+(PointXYZIR const &, PointXYZIR const &);
  PointXYZ operator-(PointXYZIR const &, PointXYZIR const &);

  PointXYZ operator-(PointXYZIR const &);

  float norm(PointXYZIR const & p);        // magnitude
  //! if norm is zero, this give invalid value
  PointXYZ normalize(PointXYZIR const &);
  float squaredNorm(PointXYZIR const &);
  float dot(PointXYZIR const &, PointXYZIR const &);
  PointXYZ cross(PointXYZIR const &, PointXYZIR const &);


} // namespace quanergy


POINT_CLOUD_REGISTER_POINT_STRUCT(quanergy::PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring))

#endif // __QUANERGY_POINT_XYZIR_H

