/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Fraction_H
#define OpenFDM_Fraction_H

#include <istream>
#include <ostream>
#include <cctype>

#include "Types.h"
#include "Limits.h"
#include "Math.h"

namespace OpenFDM {

class Fraction {
public:
  typedef long numerator_type;
  typedef unsigned long denominator_type;

  Fraction()
  { }
  Fraction(const numerator_type& numerator) :
    mNumerator(numerator),
    mDenominator(1)
  { }
  Fraction(const numerator_type& numerator,
           const denominator_type& denominator) :
    mNumerator(numerator),
    mDenominator(denominator)
  { reduce(); }

  const numerator_type& getNumerator() const
  { return mNumerator; }
  const denominator_type& getDenominator() const
  { return mDenominator; }

  real_type getRealValue() const
  { return real_type(getNumerator())/real_type(getDenominator()); }

  denominator_type reductionFactor() const
  { return gcd(std::abs(mNumerator), mDenominator); }

  Fraction& reduce()
  {
    denominator_type tmp = reductionFactor();
    if (tmp <= 1)
      return *this;
    mNumerator /= tmp;
    mDenominator /= tmp;
    return *this;
  }

  Fraction& operator+=(const Fraction& fraction)
  {
    // Just in case that the argument is *this
    numerator_type numerator = fraction.mNumerator;
    denominator_type denominator = fraction.mDenominator;

    denominator_type g = gcd(mDenominator, denominator);
    mDenominator /= g;
    mNumerator = mNumerator*(denominator/g) + numerator*mDenominator;
    g = gcd(std::abs(mNumerator), g);
    mNumerator /= g;
    mDenominator *= denominator/g;

    return *this;
  }
  Fraction& operator-=(const Fraction& fraction)
  {
    // Just in case that the argument is *this
    numerator_type numerator = fraction.mNumerator;
    denominator_type denominator = fraction.mDenominator;

    denominator_type g = gcd(mDenominator, denominator);
    mDenominator /= g;
    mNumerator = mNumerator*(denominator/g) - numerator*mDenominator;
    g = gcd(std::abs(mNumerator), g);
    mNumerator /= g;
    mDenominator *= denominator/g;

    return *this;
  }

  Fraction& operator*=(const Fraction& fraction)
  {
    // Just in case that the argument is *this
    numerator_type numerator = fraction.mNumerator;
    denominator_type denominator = fraction.mDenominator;

    denominator_type g0 = gcd(std::abs(mNumerator), denominator);
    denominator_type g1 = gcd(std::abs(numerator), mDenominator);

    mNumerator /= g0;
    mNumerator *= numerator/g1;
    mDenominator /= g1;
    mDenominator *= denominator/g0;
    return *this;
  }
  Fraction& operator/=(const Fraction& fraction)
  {
    // Just in case that the argument is *this
    numerator_type numerator = fraction.mNumerator;
    denominator_type denominator = fraction.mDenominator;

    denominator_type g0 = gcd(mDenominator, denominator);
    denominator_type g1 = gcd(std::abs(mNumerator), std::abs(numerator));

    mNumerator /= g1;
    mNumerator *= denominator/g0;
    mDenominator /= g0;
    mDenominator *= numerator/g1;
    return *this;
  }

  Fraction operator+() const
  { return *this; }

  Fraction operator-() const
  { return Fraction(-getNumerator(), getDenominator(), true); }

  static Fraction NaN() { return Fraction(0, 0, true); }
  static Fraction inf() { return Fraction(1, 0, true); }

  static denominator_type gcd(denominator_type a, denominator_type b)
  {
    if (a == 0 || b == 0)
      return 1;

    while (b) {
      denominator_type r = a % b;
      a = b;
      b = r;
    }

    return a;
  }

  static
  bool
  less(const Fraction& u, const Fraction& v)
  {
    // Avoid overflow
    denominator_type ud = u.getDenominator();
    denominator_type vd = v.getDenominator();
    denominator_type gcdd = Fraction::gcd(ud, vd);
    ud /= gcdd;
    vd /= gcdd;
    
    numerator_type un = u.getNumerator();
    numerator_type vn = v.getNumerator();
    denominator_type gcdn = gcd(std::abs(un), std::abs(vn));
    un /= gcdn;
    vn /= gcdn;
    
    return un*vd < ud*vn;
  }

  static
  bool
  equal(const Fraction& u, const Fraction& v)
  {
    // Avoid overflow
    denominator_type ud = u.getDenominator();
    denominator_type vd = v.getDenominator();
    denominator_type gcdd = Fraction::gcd(ud, vd);
    ud /= gcdd;
    vd /= gcdd;
    
    numerator_type un = u.getNumerator();
    numerator_type vn = v.getNumerator();
    denominator_type gcdn = gcd(std::abs(un), std::abs(vn));
    un /= gcdn;
    vn /= gcdn;
    
    return un*vd == ud*vn;
  }

private:
  Fraction(const numerator_type& numerator,
           const denominator_type& denominator,
           bool reduced) :
    mNumerator(numerator),
    mDenominator(denominator)
  { if (!reduced) reduce(); }

  numerator_type mNumerator;
  denominator_type mDenominator;
};

inline
Fraction
operator+(const Fraction& u, const Fraction& v)
{ return Fraction(u) += v; }

inline
Fraction
operator-(const Fraction& u, const Fraction& v)
{ return Fraction(u) -= v; }

inline
Fraction
operator*(const Fraction& u, const Fraction& v)
{ return Fraction(u) *= v; }

inline
Fraction
operator/(const Fraction& u, const Fraction& v)
{ return Fraction(u) /= v; }

inline
Fraction
operator%(const Fraction& x, const Fraction& y)
{
  Fraction q = x/y;
  Fraction::denominator_type n = std::abs(q.getNumerator())/q.getDenominator();
  return x - n*y;
}

inline
Fraction
fmod(const Fraction& x, const Fraction& y)
{ return x % y; }

inline
bool
isfinite(const Fraction& fraction)
{ return fraction.getDenominator() != 0; }

inline
bool
isnan(const Fraction& fraction)
{ return fraction.getNumerator() == 0 && fraction.getDenominator() == 0; }

inline
bool
operator==(const Fraction& u, const Fraction& v)
{
  if (isnan(u) || isnan(v))
    return false;
  return Fraction::equal(u, v);
}

inline
bool
operator!=(const Fraction& u, const Fraction& v)
{
  if (isnan(u) || isnan(v))
    return false;
  return !Fraction::equal(u, v);
}

inline
bool
operator<(const Fraction& u, const Fraction& v)
{
  if (isnan(u) || isnan(v))
    return false;
  return Fraction::less(u, v);
}

inline
bool
operator<=(const Fraction& u, const Fraction& v)
{
  if (isnan(u) || isnan(v))
    return false;
  return !Fraction::less(v, u);
}

inline
bool
operator>(const Fraction& u, const Fraction& v)
{
  if (isnan(u) || isnan(v))
    return false;
  return Fraction::less(v, u);
}

inline
bool
operator>=(const Fraction& u, const Fraction& v)
{
  if (isnan(u) || isnan(v))
    return false;
  return !Fraction::less(u, v);
}

inline
Fraction
abs(const Fraction& x)
{ return Fraction(std::abs(x.getNumerator()), x.getDenominator()); }

inline
Fraction
fabs(const Fraction& x)
{ return Fraction(std::abs(x.getNumerator()), x.getDenominator()); }


/// Compute the greatest common divisor of a and b
inline
Fraction
greatestCommonDivisor(Fraction a, Fraction b)
{
  a = abs(a);
  b = abs(b);

  while (b != 0) {
    Fraction r = a % b;
    a = b;
    b = r;
  }

  return a;
}


template<typename char_type, typename traits_type> 
inline
std::basic_ostream<char_type, traits_type>&
operator<<(std::basic_ostream<char_type, traits_type>& os, const Fraction& f)
{ return os << f.getNumerator() << os.widen('/') << f.getDenominator(); }

template<typename char_type, typename traits_type> 
inline
std::basic_istream<char_type, traits_type>&
operator>>(std::basic_istream<char_type, traits_type>& is, Fraction& f)
{
  Fraction::numerator_type numerator;
  is >> numerator;
  if (is.peek() == is.widen('/')) {
    is.ignore(1);
    Fraction::denominator_type denominator;
    is >> denominator;
    f = Fraction(numerator, denominator);
    return is;

  } else if (is.peek() == is.widen('.')) {
    is.ignore(1);
    
    Fraction::denominator_type denominator = 1;
    while (std::isdigit(is.peek())) {
      char_type digit = is.get();
      numerator = numerator*10 + digit - '0';
      denominator *= 10;
    }
    f = Fraction(numerator, denominator);
    
    return is;
  } else {
    f = Fraction(numerator);
    return is;
  }
}

} // namespace OpenFDM

#endif
