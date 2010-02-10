/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich
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
                    signed char time = 0,
                    signed char thermodynamicTemperature = 0,
                    signed char electricCurrent = 0,
                    signed char amountOfSubstance = 0,
                    signed char luminousIntensity = 0) :
    mLength(length),
    mMass(mass),
    mTime(time),
    mThermodynamicTemperature(thermodynamicTemperature),
    mElectricCurrent(electricCurrent),
    mAmountOfSubstance(amountOfSubstance),
    mLuminousIntensity(luminousIntensity)
  { }

  signed char getLength(void) const
  { return mLength; }
  signed char getMass(void) const
  { return mMass; }
  signed char getTime(void) const
  { return mTime; }
  signed char getThermodynamicTemperature(void) const
  { return mThermodynamicTemperature; }
  signed char getElectricCurrent(void) const
  { return mElectricCurrent; }
  signed char getAmountOfSubstance(void) const
  { return mAmountOfSubstance; }
  signed char getLuminousIntensity(void) const
  { return mLuminousIntensity; }

  static PhysicalDimension length(void)
  { return PhysicalDimension(1, 0, 0, 0, 0, 0, 0); }
  static PhysicalDimension mass(void)
  { return PhysicalDimension(0, 1, 0, 0, 0, 0, 0); }
  static PhysicalDimension time(void)
  { return PhysicalDimension(0, 0, 1, 0, 0, 0, 0); }
  static PhysicalDimension thermodynamicTemperature(void)
  { return PhysicalDimension(0, 0, 0, 1, 0, 0, 0); }
  static PhysicalDimension electricCurrent(void)
  { return PhysicalDimension(0, 0, 0, 0, 1, 0, 0); }
  static PhysicalDimension amountOfSubstance(void)
  { return PhysicalDimension(0, 0, 0, 0, 0, 1, 0); }
  static PhysicalDimension luminousIntensity(void)
  { return PhysicalDimension(0, 0, 0, 0, 0, 0, 1); }

  static PhysicalDimension planeAngle(void)
  { return length()/length(); }
  static PhysicalDimension solidAngle(void)
  { return area()/area(); }

  static PhysicalDimension frequency(void)
  { return PhysicalDimension()/time(); }

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

  static PhysicalDimension electricCharge(void)
  { return electricCurrent()*time(); }

  PhysicalDimension& operator*=(const PhysicalDimension& physicalDimension)
  { *this = *this * physicalDimension; return *this; }
  PhysicalDimension& operator/=(const PhysicalDimension& physicalDimension)
  { *this = *this / physicalDimension; return *this; }

private:
  signed char mLength;
  signed char mMass;
  signed char mTime;
  signed char mThermodynamicTemperature;
  signed char mElectricCurrent;
  signed char mAmountOfSubstance;
  signed char mLuminousIntensity;
};

inline
PhysicalDimension
operator*(const PhysicalDimension& d1, const PhysicalDimension& d2)
{
  return PhysicalDimension(d1.getLength()+d2.getLength(),
                           d1.getMass()+d2.getMass(),
                           d1.getTime()+d2.getTime(),
                           d1.getThermodynamicTemperature()
                           +d2.getThermodynamicTemperature(),
                           d1.getElectricCurrent()+d2.getElectricCurrent(),
                           d1.getAmountOfSubstance()+d2.getAmountOfSubstance(),
                           d1.getLuminousIntensity()+d2.getLuminousIntensity());
}

inline
PhysicalDimension
operator/(const PhysicalDimension& d1, const PhysicalDimension& d2)
{
  return PhysicalDimension(d1.getLength()-d2.getLength(),
                           d1.getMass()-d2.getMass(),
                           d1.getTime()-d2.getTime(),
                           d1.getThermodynamicTemperature()
                           -d2.getThermodynamicTemperature(),
                           d1.getElectricCurrent()-d2.getElectricCurrent(),
                           d1.getAmountOfSubstance()-d2.getAmountOfSubstance(),
                           d1.getLuminousIntensity()-d2.getLuminousIntensity());
}

inline
bool
operator==(const PhysicalDimension& d1, const PhysicalDimension& d2)
{
  return d1.getLength() == d2.getLength() &&
    d1.getMass() == d2.getMass() &&
    d1.getTime() == d2.getTime() &&
    d1.getThermodynamicTemperature() == d2.getThermodynamicTemperature() &&
    d1.getElectricCurrent() == d2.getElectricCurrent() &&
    d1.getAmountOfSubstance() == d2.getAmountOfSubstance() &&
    d1.getLuminousIntensity() == d2.getLuminousIntensity();
}

inline
bool
operator!=(const PhysicalDimension& d1, const PhysicalDimension& d2)
{
  return d1.getLength() != d2.getLength() ||
    d1.getMass() != d2.getMass() ||
    d1.getTime() != d2.getTime() ||
    d1.getThermodynamicTemperature() != d2.getThermodynamicTemperature() ||
    d1.getElectricCurrent() != d2.getElectricCurrent() ||
    d1.getAmountOfSubstance() != d2.getAmountOfSubstance() ||
    d1.getLuminousIntensity() != d2.getLuminousIntensity();
}

inline
PhysicalDimension
pow(const PhysicalDimension& d, signed char exponent)
{
  if (exponent == 0)
    return PhysicalDimension();
  else if (exponent == 1)
    return d;
  else if (1 < exponent)
    return d*pow(d, exponent-1);
  else if (exponent < 0)
    return PhysicalDimension()/pow(d, -exponent);
}

} // namespace OpenFDM

#endif
