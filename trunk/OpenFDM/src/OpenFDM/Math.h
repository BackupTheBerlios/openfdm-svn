/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Math_H
#define OpenFDM_Math_H

#include <cmath>
#include <algorithm>
#include "Types.h"
#include "Limits.h"

namespace OpenFDM {

using std::abs;
using std::acos;
using std::asin;
using std::atan;
using std::atan2;
using std::ceil;
using std::cos;
using std::cosh;
using std::exp;
using std::fabs;
using std::floor;
using std::fmod;
using std::frexp;
using std::ldexp;
using std::log;
using std::log10;
using std::pow;
using std::sin;
using std::sinh;
using std::sqrt;
using std::tan;
using std::tanh;

using std::fpclassify;
using std::isfinite;
using std::isinf;
using std::isnan;
using std::isnormal;
using std::signbit;
using std::isgreater;
using std::isgreaterequal;
using std::isless;
using std::islessequal;
using std::islessgreater;
using std::isunordered;

using std::min;
using std::max;

template<typename T>
inline int
sign(const T& val)
{
  if (val <= -Limits<T>::min())
    return -1;
  if (Limits<T>::min() <= val)
    return 1;
  return 0;
}

template<typename T>
inline T
saturate(const T& val, const T& saturation)
{
  if (val <= -saturation)
    return -saturation;
  if (saturation <= val)
    return saturation;
  return val;
}

template<typename T>
inline T
deadBand(const T& val, const T& saturation)
{
  if (val <= -saturation)
    return val + saturation;
  if (saturation <= val)
    return val - saturation;
  return T(0);
}

template<typename T>
inline T
smoothSaturate(const T& val, const T& saturation)
{
  /// FIXME
#define PI static_cast<T>(3.1415926535897932384626433832795029L)
  if (saturation < Limits<T>::min())
    return 0;
  else
    return atan(val*0.5*PI/saturation)*2*saturation/PI;
#undef PI
}

template<typename T>
inline T
smoothDeadBand(const T& val, const T& saturation)
{
  return val - smoothSaturate(val, saturation);
}

template<typename S, typename T>
inline S
interpolate(const T& x, const T& x0, const S& y0, T x1, const S& y1)
{
  // If called in the wrong order, simply call with the correct order ...
  if (x1 < x0)
    return interpolate(x, x1, y1, x0, y0);
  
  // Important to have <'=':
  // Often used to blend away from a division by zero.
  if (x <= x0) {
    return y0;
  } else if (x < x1) {
    T theta = (x-x0)/(x1-x0);
    return theta*y1+(1.0-theta)*y0;
  } else {
    return y1;
  }
}

/// Compute the greatest common divisor of a and b
/// Floating point version, might provide problems with roundoff ...
inline
real_type
greatestCommonDivisor(real_type a_, real_type b_)
{
  real_type a = fabs(a_);
  real_type b = fabs(b_);

  real_type eps = 64*max(a, b)*Limits<real_type>::epsilon();
  unsigned opcount = 1;
  while (eps*opcount < b) {
    real_type r = fmod(a, b);
    a = b;
    b = r;
    ++opcount;
  }

  // should return a, but make it a natural numbered quotient as much
  // as possible
//   std::cout << a - a_/floor(a_/a + 0.5) << std::endl;
//   return a;
  return fabs(a_)/floor(fabs(a_)/a + 0.5);
}

/// Returns true if the floatingpoint values a and b are equal to roundoff
/// times a security factor sf.
inline
bool
equal(real_type a, real_type b, real_type sf)
{ return fabs(a-b) < max(fabs(a), fabs(b))*sf*Limits<real_type>::epsilon(); }

/// Returns true if the floatingpoint values are equal to roundoff with a
/// security factor of 8.
inline
bool
equal(real_type a, real_type b)
{ return equal(a, b, (real_type)8); }

} // namespace OpenFDM

#endif
