/*************************************************
 **                                             **
 **  Copyright 2014-2015 Quanergy Systems, Inc  **
 **  All Rights Reserved.                       **
 **                                             **
 *************************************************/

/** \file
 *
 *  Point Cloud Library point structures for cartesian data.
 *
 */

#ifndef QUANERGY_COMMON_POINT_XYZ_H
#define QUANERGY_COMMON_POINT_XYZ_H

#include <ostream>

#include <pcl/point_types.h>


namespace quanergy
{
  struct EIGEN_ALIGN16 PointXYZ
  {
    PCL_ADD_POINT4D;

    PointXYZ (const PointXYZ& p) 
      : x(p.x)
      , y(p.y)
      , z(p.z)
    {
      data[3] = 1.f;
    }

    PointXYZ ()
      : x(0.0f)
      , y(0.0f)
      , z(0.0f)
    {
      data[3] = 1.f;
    }

    PointXYZ (float _x, float _y, float _z)
      : x(_x)
      , y(_y)
      , z(_z)
    {
      data[3] = 1.f;
    }

    friend std::ostream& operator<< (std::ostream &out, const PointXYZ &point);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace quanergy


inline std::ostream& operator<< (std::ostream &out, const quanergy::PointXYZ &point)
{
  out << point.x << " " << point.y << " " << point.z;
  return out;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(quanergy::PointXYZ,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z))

PointXYZ operator+(PointXYZ const &, float);
PointXYZ operator-(PointXYZ const &, float);
PointXYZ operator*(PointXYZ const &, float);
PointXYZ operator/(PointXYZ const &, float);
PointXYZ operator+(PointXYZ const &, PointXYZ const &);
PointXYZ operator-(PointXYZ const &, PointXYZ const &);

float norm(PointXYZ const & p);        // magnitude
PointXYZ normalize(PointXYZ const &);
float squaredNorm(PointXYZ const &);
float dot (PointXYZ const &, PointXYZ const &);
PointXYZ cross (PointXYZ const &, PointXYZ const &)

#endif
