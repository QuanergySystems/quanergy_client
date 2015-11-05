#include <quanergy/common/point_xyz.h>

#include <cmath>

#if defined(__SSE__) && defined(__SSE4_1__)

#include <xmmintrin.h>
#include <smmintrin.h>


THIS_POINT_TYPE operator+ (const THIS_POINT_TYPE& lhs, float alpha) \
  {                                                                     \
    __m128 sse_alpha = _mm_set_ps1 (alpha);                             \
    __m128 sse_lhs = _mm_load_ps (&(lhs.x));                            \
    sse_alpha = _mm_add_ps (sse_lhs, sse_alpha);                        \
    THIS_POINT_TYPE add;                                                \
    _mm_store_ps (&(add.x), sse_alpha);                                 \
    return (add);                                                       \
  }                                                                     \
  inline THIS_POINT_TYPE operator- (const THIS_POINT_TYPE& lhs, float alpha) \
  {                                                                     \
    __m128 sse_alpha = _mm_set_ps1 (alpha);                             \
    __m128 sse_lhs = _mm_load_ps (&(lhs.x));                            \
    sse_alpha = _mm_sub_ps (sse_lhs, sse_alpha);                        \
    THIS_POINT_TYPE sub;                                                \
    _mm_store_ps (&(sub.x), sse_alpha);                                 \
    return (sub);                                                       \
  }                                                                     \
  inline THIS_POINT_TYPE operator* (const THIS_POINT_TYPE& lhs, float alpha) \
  {                                                                     \
    __m128 sse_alpha = _mm_set_ps1 (alpha);                             \
    __m128 sse_lhs = _mm_load_ps (&(lhs.x));                            \
    sse_alpha = _mm_mul_ps (sse_lhs, sse_alpha);                        \
    THIS_POINT_TYPE mul;                                                \
    _mm_store_ps (&(mul.x), sse_alpha);                                 \
    return (mul);                                                       \
  }                                                                     \
  inline THIS_POINT_TYPE operator/ (const THIS_POINT_TYPE& lhs, float alpha) \
  {                                                                     \
    __m128 sse_alpha = _mm_set_ps1 (alpha);                             \
    __m128 sse_lhs = _mm_load_ps (&(lhs.x));                            \
    sse_alpha = _mm_div_ps (sse_lhs, sse_alpha);                        \
    THIS_POINT_TYPE div;                                                \
    _mm_store_ps (&(div.x), sse_alpha);                                 \
    return (div);                                                       \
  }                                                                     \
  inline THIS_POINT_TYPE operator+ (const THIS_POINT_TYPE& lhs, const THIS_POINT_TYPE& rhs) \
  {                                                                     \
    __m128 sse_lhs = _mm_load_ps (&(lhs.x));                            \
    __m128 sse_rhs = _mm_load_ps (&(rhs.x));                            \
    sse_rhs = _mm_add_ps (sse_lhs, sse_rhs);                            \
    THIS_POINT_TYPE add;                                                \
    _mm_store_ps (&(add.x), sse_rhs);                                   \
    return (add);                                                       \
  }                                                                     \
  inline THIS_POINT_TYPE operator- (const THIS_POINT_TYPE& lhs, const THIS_POINT_TYPE& rhs) \
  {                                                                     \
    __m128 sse_lhs = _mm_load_ps (&(lhs.x));                            \
    __m128 sse_rhs = _mm_load_ps (&(rhs.x));                            \
    sse_rhs = _mm_sub_ps (sse_lhs, sse_rhs);                            \
    THIS_POINT_TYPE sub;                                                \
    _mm_store_ps (&(sub.x), sse_rhs);                                   \
    return (sub);                                                       \
  }                                                                     \
  template<typename AnotherPoint>                                       \
  inline THIS_POINT_TYPE operator+ (const THIS_POINT_TYPE& lhs, const AnotherPoint& rhs) \
  {                                                                     \
    __m128 sse_lhs = _mm_load_ps (&(lhs.x));                            \
    __m128 sse_rhs = _mm_load_ps (&(rhs.x));                            \
    sse_rhs = _mm_add_ps (sse_lhs, sse_rhs);                            \
    THIS_POINT_TYPE add;                                                \
    _mm_store_ps (&(add.x), sse_rhs);                                   \
    return (add);                                                       \
  }                                                                     \
  template<typename AnotherPoint>                                       \
  inline THIS_POINT_TYPE operator- (const THIS_POINT_TYPE& lhs, const AnotherPoint& rhs) \
  {                                                                     \
    __m128 sse_lhs = _mm_load_ps (&(lhs.x));                            \
    __m128 sse_rhs = _mm_load_ps (&(rhs.x));                            \
    sse_rhs = _mm_sub_ps (sse_lhs, sse_rhs);                            \
    THIS_POINT_TYPE sub;                                                \
    _mm_store_ps (&(sub.x), sse_rhs);                                   \
    return (sub);                                                       \
  }

#else


PointXYZ operator+(PointXYZ const & lhs, float alpha)
{
  return PointXYZ(lhs.x+alpha, lhs.y+alpha, lhs.z+alpha);
}


PointXYZ operator-(PointXYZ const & lhs, float alpha)
{
  return PointXYZ(lhs.x-alpha, lhs.y-alpha, lhs.z-alpha);
}


PointXYZ operator*(PointXYZ const & lhs, float alpha)
{
  return PointXYZ(lhs.x*alpha, lhs.y*alpha, lhs.z*alpha);
}


PointXYZ operator/(PointXYZ const & lhs, float alpha)
{
  return PointXYZ(lhs.x/alpha, lhs.y/alpha, lhs.z/alpha);
}


PointXYZ operator+(PointXYZ const & lhs, PointXYZ const & rhs)
{
  return PointXYZ(lhs.x+rhs.x, lhs.y+rhs.y, lhs.z+rhs.z);
}


PointXYZ operator-(PointXYZ const & lhs, PointXYZ const & rhs)
{
  return PointXYZ(lhs.x-rhs.x, lhs.y-rhs.y, lhs.z-rhs.z);
}


float norm (PointXYZ const & p)
{
  return std::sqrt<float>(x*x + y*y + z*z);
}


PointXYZ normalize (PointXYZ const & p)
{
  float norm = std::sqrt<float>(p.x*p.x + p.y*p.y + p.z*p.z);

  float s = 1.0f / norm;

  return p * s;
}


float squaredNorm (PointXYZ const & p)
{
  return (x*x + y*y + z*z);
}


float dot (PointXYZ const & a, PointXYZ const & b)
{
  return (a.x*b.x + a.y*b.y + a.z*b.z);
}


PointXYZ cross(PointXYZ const & a, PointXYZ const & b)
{
  return PointXYZ(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}


#endif
