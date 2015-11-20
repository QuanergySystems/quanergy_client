/****************************************************************
 **                                                            **
 **  Copyright(C) 2015 Quanergy Systems. All Rights Reserved.  **
 **  Contact: http://www.quanergy.com                          **
 **                                                            **
 ****************************************************************/

#include <quanergy/common/point_xyzir.h>

#include <cmath>

namespace quanergy
{

PointXYZ operator+(PointXYZIR const & lhs, float alpha)
{
  return PointXYZ(lhs.x+alpha, lhs.y+alpha, lhs.z+alpha);
}


PointXYZ operator-(PointXYZIR const & lhs, float alpha)
{
  return PointXYZ(lhs.x-alpha, lhs.y-alpha, lhs.z-alpha);
}


PointXYZ operator*(PointXYZIR const & lhs, float alpha)
{
  return PointXYZ(lhs.x*alpha, lhs.y*alpha, lhs.z*alpha);
}


PointXYZ operator/(PointXYZIR const & lhs, float alpha)
{
  return PointXYZ(lhs.x/alpha, lhs.y/alpha, lhs.z/alpha);
}


PointXYZ operator+(PointXYZIR const & lhs, PointXYZIR const & rhs)
{
  return PointXYZ(lhs.x+rhs.x, lhs.y+rhs.y, lhs.z+rhs.z);
}


PointXYZ operator-(PointXYZIR const & lhs, PointXYZIR const & rhs)
{
  return PointXYZ(lhs.x-rhs.x, lhs.y-rhs.y, lhs.z-rhs.z);
}

PointXYZ operator-(PointXYZIR const & p)
{
  return PointXYZ(-p.x, -p.y, -p.z);
}


float norm(PointXYZIR const & p)
{
  return std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


PointXYZ normalize(PointXYZIR const & p)
{
  float norm = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);

  float s = 1.0f / norm;

  return p * s;
}


float squaredNorm(PointXYZIR const & p)
{
  return (p.x*p.x + p.y*p.y + p.z*p.z);
}


float dot (PointXYZIR const & a, PointXYZIR const & b)
{
  return (a.x*b.x + a.y*b.y + a.z*b.z);
}


PointXYZ cross(PointXYZIR const & a, PointXYZIR const & b)
{
  return PointXYZ(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}

} // namespace quanergy
