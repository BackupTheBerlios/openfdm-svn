/* -*-c++-*- OpenFDM - Copyright (C) 2004-2007 Mathias Froehlich
 *
 */

#ifndef OpenFDM_PhysicalDimension_H
#define OpenFDM_PhysicalDimension_H

#include <string>

#include "LogStream.h"
#include "Types.h"
#include "Vector.h"

namespace OpenFDM {

class PhysicalDimension;

inline PhysicalDimension
operator*(const PhysicalDimension& d1, const PhysicalDimension& d2);
inline PhysicalDimension
operator/(const PhysicalDimension& d1, const PhysicalDimension& d2);

class PhysicalDimension {
public:
  PhysicalDimension(signed char length = 0, signed char mass = 0,
                    signed char time = 0, signed char temperature = 0,
                    signed char current = 0, signed char amountSubstance = 0,
                    signed char lumIntensity = 0) :
    mLength(length),
    mMass(mass),
    mTime(time),
    mTemperature(temperature),
    mCurrent(current),
    mAmountSubstance(amountSubstance),
    mLumIntensity(lumIntensity)
  { }

  signed char getLength(void) const { return mLength; }
  signed char getMass(void) const { return mMass; }
  signed char getTime(void) const { return mTime; }
  signed char getTemperature(void) const { return mTemperature; }
  signed char getCurrent(void) const { return mCurrent; }
  signed char getAmountSubstance(void) const { return mAmountSubstance; }
  signed char getLumIntensity(void) const { return mLumIntensity; }

  static PhysicalDimension length(void)
  { return PhysicalDimension(1, 0, 0, 0, 0, 0, 0); }
  static PhysicalDimension mass(void)
  { return PhysicalDimension(0, 1, 0, 0, 0, 0, 0); }
  static PhysicalDimension time(void)
  { return PhysicalDimension(0, 0, 1, 0, 0, 0, 0); }
  static PhysicalDimension temperature(void)
  { return PhysicalDimension(0, 0, 0, 1, 0, 0, 0); }
  static PhysicalDimension current(void)
  { return PhysicalDimension(0, 0, 0, 0, 1, 0, 0); }
  static PhysicalDimension amountSubstance(void)
  { return PhysicalDimension(0, 0, 0, 0, 0, 1, 0); }
  static PhysicalDimension lumIntensity(void)
  { return PhysicalDimension(0, 0, 0, 0, 0, 0, 1); }
  static PhysicalDimension area(void)
  { return PhysicalDimension(2, 0, 0, 0, 0, 0, 0); }
  static PhysicalDimension volume(void)
  { return PhysicalDimension(3, 0, 0, 0, 0, 0, 0); }
  static PhysicalDimension velocity(void)
  { return length()/time(); }
  static PhysicalDimension acceleration(void)
  { return velocity()/time(); }
  static PhysicalDimension jerk(void)
  { return acceleration()/time(); }
  static PhysicalDimension force(void)
  { return mass()*acceleration(); }
  static PhysicalDimension pressure(void)
  { return force()/area(); }
  static PhysicalDimension density(void)
  { return mass()/volume(); }
  static PhysicalDimension energy(void)
  { return force()*length(); }
  static PhysicalDimension power(void)
  { return energy()/time(); }
  static PhysicalDimension inertia(void)
  { return mass()*area(); }

  PhysicalDimension& operator*=(const PhysicalDimension& physicalDimension)
  { *this = *this * physicalDimension; return *this; }
  PhysicalDimension& operator/=(const PhysicalDimension& physicalDimension)
  { *this = *this / physicalDimension; return *this; }

private:
  signed char mLength;
  signed char mMass;
  signed char mTime;
  signed char mTemperature;
  signed char mCurrent;
  signed char mAmountSubstance;
  signed char mLumIntensity;
};

inline
PhysicalDimension
operator*(const PhysicalDimension& d1, const PhysicalDimension& d2)
{
  return PhysicalDimension(d1.getLength()+d2.getLength(),
                           d1.getMass()+d2.getMass(),
                           d1.getTime()+d2.getTime(),
                           d1.getTemperature()+d2.getTemperature(),
                           d1.getCurrent()+d2.getCurrent(),
                           d1.getAmountSubstance()+d2.getAmountSubstance(),
                           d1.getLumIntensity()+d2.getLumIntensity());
}

inline
PhysicalDimension
operator/(const PhysicalDimension& d1, const PhysicalDimension& d2)
{
  return PhysicalDimension(d1.getLength()-d2.getLength(),
                           d1.getMass()-d2.getMass(),
                           d1.getTime()-d2.getTime(),
                           d1.getTemperature()-d2.getTemperature(),
                           d1.getCurrent()-d2.getCurrent(),
                           d1.getAmountSubstance()-d2.getAmountSubstance(),
                           d1.getLumIntensity()-d2.getLumIntensity());
}

inline
bool
operator==(const PhysicalDimension& d1, const PhysicalDimension& d2)
{
  return d1.getLength() == d2.getLength() &&
    d1.getMass() == d2.getMass() &&
    d1.getTime() == d2.getTime() &&
    d1.getTemperature() == d2.getTemperature() &&
    d1.getCurrent() == d2.getCurrent() &&
    d1.getAmountSubstance() == d2.getAmountSubstance() &&
    d1.getLumIntensity() == d2.getLumIntensity();
}

inline
bool
operator!=(const PhysicalDimension& d1, const PhysicalDimension& d2)
{
  return d1.getLength() != d2.getLength() ||
    d1.getMass() != d2.getMass() ||
    d1.getTime() != d2.getTime() ||
    d1.getTemperature() != d2.getTemperature() ||
    d1.getCurrent() != d2.getCurrent() ||
    d1.getAmountSubstance() != d2.getAmountSubstance() ||
    d1.getLumIntensity() != d2.getLumIntensity();
}

inline
PhysicalDimension
exp(const PhysicalDimension& d, signed char exponent)
{
  if (exponent == 0)
    return PhysicalDimension();
  else if (exponent == 1)
    return d;
  else if (1 < exponent)
    return d*exp(d, exponent-1);
  else if (exponent < 0)
    return PhysicalDimension()/exp(d, -exponent);
}

} // namespace OpenFDM

#endif
