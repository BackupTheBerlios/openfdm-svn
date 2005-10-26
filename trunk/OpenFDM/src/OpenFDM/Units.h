/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich
 *
 */

#ifndef OpenFDM_Units_H
#define OpenFDM_Units_H

#include <iosfwd>
#include <string>

#include "Types.h"
#include "Vector.h"

namespace OpenFDM {

/**
   OpenFDM does all internal computations in the SI system.
   But configurations files or table lookups are often done in different
   unit systems. This modules provides the unit conversion routines together
   with some basic mathematical or physical constants.

   Only compatible units could be converted. That means that it is
   sufficient to provied a source or target unit you want to convert a
   value from or to. Given that one there is one well defined SI unit
   where it is converted to or from.

   There is a parsing helper routine which determines a unit from a
   string. This parsing routine understands the units like described
   in a recent Dave-ML draft available from

   http://daveml.nasa.gov/Contributions/FltSysUnitStandard.html

   The following table is taken from there and describes the unit abreviations
   valid for the parsing routine.

   Powers of units are written with the power just written behind the
   unit, for example acceleration in the SI system is 'm s-2'.

Measure                         Abbreviation    Comments

  General
     non-dimensional            ND              non-dimensional

  Prefix 2
     tera                       T               1e12 * parameter
     giga                       G               1e9 * parameter
     mega                       M               1e6 * parameter
     kilo                       k               1e3 * parameter
     deci                       d               1e-1 * parameter
     centi                      c               1e-2 * parameter
     milli                      m               1e-3 * parameter

  Time
     Solar year                 yr
     Solar day                  day
     hour                       h
     minute                     min
     second                     s               SI Standard

  Length
     metres                     m               SI Standard
     inch                       in
     feet                       ft
     yard                       yd
     nautical mile              nmi
     statute mile               smi

  Velocity 5
     knot                       kn              nmi h-1

  Volume
     litre                      l               10-3 m3
     US gallon                  USgal
     UK gallon                  UKgal           Imperial Gallon

  Mass
     kilogram                   kg              SI Standard -- 103g
     gram                       g
     tonne                      tonne
     slug                       slug
     short ton                  USton           US Ton
     long ton                   UKton           UK Ton
     force at sealevel          lbs             not a mass as such, but handy

  Force
     Newton                     N               SI Standard
     Pound Force                lbf

  Pressure
     Pascal                     Pa              N m-2
     millimetres Mercury        mmHg
     Pounds per square foot     lbf ft-2        lbf ft-2
     Pounds per square inch     lbf in-2        lbf in-2
     inches Mercury             inHg
     Atmosphere                 atm

  Temperature
     degrees Kelvin             K               SI Standard
     degrees Centigrade         C
     degrees Farenheight        F
     degrees Rankin             R

  Viscosity
     Poise                      Ps              Dynamic viscosity
     Stokes                     St              Kinematic viscosity

  Plane Angle
     degree                     deg
     radian                     rad
     revolution                 rev

  Power, Energy
     Watt                       W               SI Standard
     horsepower                 hp              550 ft lbf s-1
     joule                      jou             SI Standard
     British thermal unit       btu
     calorie                    cal
     erg                        erg

  Electrical, Magnetic
     volt direct current        Vdc
     volt alternating current   Vac
     ampere                     A               SI Standard
     omh, resistance            ohm             SI Standard
     cycle                      cyc
     henry                      hy
     farad                      fd
     Telsa                      T               field strength

 */

/** Just pi*2 */
extern const real_type pi2;
/** Just pi */
extern const real_type pi;
/** Just pi*0.5 */
extern const real_type pi05;
/** Just pi*0.25 */
extern const real_type pi025;

extern const real_type deg2rad;
extern const real_type rad2deg;

extern const real_type gravity_constant;

enum Unit {
  // Angle 'unit'
  uRadian,
  uDegree,

  // Time unit
  uSecond,
  uMinute,
  uHour,

  // Length units.
  uMeter,
  uFoot,
  uInch,
  uMile,
  uNauticalMile,

  // Area units.
  uMeter2,
  uFoot2,

  // Speed units.
  uMeterPSecond,
  uKiloMeterPHour,
  uFeetPSecond,
  uFeetPMinute,
  uKnots,

  // Acceleration units.
  uMeterPSec2,
  uFeetPSec2,

  // Force units.
  uNewton,
  uPoundForce,

  // Energy units (moments).
  uNewtonMeter,
  uPoundForceFt,

  // Mass units.
  uKilogram,
  uPoundSealevel,
  uSlug,

  // Inertia units.
  uKilogramMeter2,
  uSlugFt2,

  // Temperature units.
  uKelvin,
  uDegC,
  uRankine,
  uFahrenheit,

  // Spring stiffness.
  uNewtonPMeter,
  uPoundForcePFt,
  uPoundForcePInch,

  // Pressure units.
  uPascal,
  uPoundPFt2,

  // Density units.
  uKilogramPMeter3,
  uSlugPFt3,

  // Prefixes.
  uTerra = 0x10000,
  uGiga,
  uMega,
  uKilo,
  uDeci,
  uCenti,
  uMilli,
  uMicro,
  uNano,

  // Hmm ...
  uUnitMask = 0x10000 - 1,
  uPrefixMask = ~(0x10000 - 1),
  uUnknown = 0
};

class PhysicalDimension;

PhysicalDimension
operator*(const PhysicalDimension& d1, const PhysicalDimension& d2);
PhysicalDimension
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
exp(const PhysicalDimension& d, int exponent)
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

class Unit2;

Unit2 operator*(const Unit2& u1, const Unit2& u2);
Unit2 operator/(const Unit2& u1, const Unit2& u2);

class Unit2 {
public:
  Unit2(void) :
    mFactor(1),
    mOffset(0)
  { }
  Unit2(real_type factor) :
    mFactor(factor),
    mOffset(0)
  { }
  Unit2(const PhysicalDimension& physDim) :
    mPhysicalDimension(physDim),
    mFactor(1),
    mOffset(0)
  { }
  Unit2(const PhysicalDimension& physDim, real_type factor) :
    mPhysicalDimension(physDim),
    mFactor(factor),
    mOffset(0)
  { }
  Unit2(const PhysicalDimension& physDim, real_type factor, real_type offset) :
    mPhysicalDimension(physDim),
    mFactor(factor),
    mOffset(offset)
  { }

  real_type getFactor(void) const
  { return mFactor; }
  real_type getOffset(void) const
  { return mOffset; }
  const PhysicalDimension& getPhysicalDimension(void) const
  { return mPhysicalDimension; }
  int getLength(void) const
  { return mPhysicalDimension.getLength(); }
  int getMass(void) const
  { return mPhysicalDimension.getMass(); }
  int getTime(void) const
  { return mPhysicalDimension.getTime(); }
  int getTemperature(void) const
  { return mPhysicalDimension.getTemperature(); }
  int getCurrent(void) const
  { return mPhysicalDimension.getCurrent(); }
  int getAmountSubstance(void) const
  { return mPhysicalDimension.getAmountSubstance(); }
  int getLumIntensity(void) const
  { return mPhysicalDimension.getLumIntensity(); }

  static Unit2 dimless;

  static Unit2 radian;
  static Unit2 degree;

  static Unit2 second;
  static Unit2 minute;
  static Unit2 hour;
  static Unit2 day;
  static Unit2 week;

  static Unit2 meter;
  static Unit2 kilometer;
  static Unit2 feet;
  static Unit2 inch;
  static Unit2 nauticalMile;
  static Unit2 statuteMile;

  static Unit2 meterPerSecond;
  static Unit2 kiloMeterPerHour;
  static Unit2 feetPerSecond;
  static Unit2 feetPerMinute;
  static Unit2 knots;

  static Unit2 meterPerSecond2;

  static Unit2 kilogramm;
  static Unit2 poundMass;
  static Unit2 lbs;
  static Unit2 slugs;

  static Unit2 N;
  static Unit2 poundForce;
  static Unit2 lbf;

  static Unit2 Pa;

  static Unit2 kelvin;
  static Unit2 degreeCelsius;
  static Unit2 rankine;
  static Unit2 degreeFarenheit;

  static Unit2 length(real_type factor = 1)
  { return Unit2(PhysicalDimension::length(), factor); }
  static Unit2 mass(real_type factor = 1)
  { return Unit2(PhysicalDimension::mass(), factor); }
  static Unit2 time(real_type factor = 1)
  { return Unit2(PhysicalDimension::time(), factor); }
  static Unit2 temperature(real_type factor = 1, real_type offset = 0)
  { return Unit2(PhysicalDimension::temperature(), factor, offset); }
  static Unit2 current(real_type factor = 1)
  { return Unit2(PhysicalDimension::current(), factor); }
  static Unit2 amountSubstance(real_type factor = 1)
  { return Unit2(PhysicalDimension::amountSubstance(), factor); }
  static Unit2 lumIntensity(real_type factor = 1)
  { return Unit2(PhysicalDimension::lumIntensity(), factor); }
  static Unit2 area(real_type factor = 1)
  { return Unit2(PhysicalDimension::area(), factor); }
  static Unit2 volume(real_type factor = 1)
  { return Unit2(PhysicalDimension::volume(), factor); }
  static Unit2 velocity(real_type factor = 1)
  { return length(factor)/time(); }
  static Unit2 acceleration(real_type factor = 1)
  { return velocity(factor)/time(); }
  static Unit2 jerk(real_type factor = 1)
  { return acceleration(factor)/time(); }
  static Unit2 force(real_type factor = 1)
  { return mass(factor)*acceleration(); }
  static Unit2 pressure(real_type factor = 1)
  { return force(factor)/area(); }
  static Unit2 density(real_type factor = 1)
  { return mass(factor)/volume(); }
  static Unit2 energy(real_type factor = 1)
  { return force(factor)*length(); }
  static Unit2 power(real_type factor = 1)
  { return energy(factor)/time(); }
  static Unit2 inertia(real_type factor = 1)
  { return mass(factor)*area(); }

private:
  real_type mFactor;
  real_type mOffset;
  PhysicalDimension mPhysicalDimension;
};

inline
Unit2
operator*(const Unit2& u1, const Unit2& u2)
{
  return Unit2(u1.getPhysicalDimension()*u2.getPhysicalDimension(),
               u1.getFactor()*u2.getFactor(),
               u1.getOffset()+u1.getFactor()*u2.getOffset());
}

inline
Unit2
operator/(const Unit2& u1, const Unit2& u2)
{
  return Unit2(u1.getPhysicalDimension()/u2.getPhysicalDimension(),
               u1.getFactor()/u2.getFactor(),
               u1.getOffset()/u2.getFactor()-u2.getOffset());
}

inline
Unit2
exp(const Unit2& u, int exponent)
{
  if (exponent == 0)
    return Unit2();
  else if (exponent == 1)
    return u;
  else if (1 < exponent)
    return u*exp(u, exponent-1);
  else if (exponent < 0)
    return Unit2()/exp(u, -exponent);
}

inline
bool
operator==(const Unit2& u1, const Unit2& u2)
{
  return u1.getFactor() == u2.getFactor() &&
    u1.getOffset() == u2.getOffset() &&
    u1.getPhysicalDimension() == u2.getPhysicalDimension();
}

inline
bool
operator!=(const Unit2& u1, const Unit2& u2)
{
  return u1.getFactor() != u2.getFactor() ||
    u1.getOffset() != u2.getOffset() ||
    u1.getPhysicalDimension() != u2.getPhysicalDimension();
}

inline
bool
compatible(const Unit2& u1, const Unit2& u2)
{
  return u1.getPhysicalDimension() == u2.getPhysicalDimension();
}

template<typename char_type, typename traits_type> 
inline
std::basic_ostream<char_type, traits_type>&
operator<<(std::basic_ostream<char_type, traits_type>& stream, const Unit2& u)
{
  stream << "Unit: Factor = " << u.getFactor()
         << ", Offset = " << u.getOffset()
         << " ";

  // Distinguish between offset free and units with offset.
  if (u.getOffset() == 0) {
    if (u.getMass()) {
      stream << "kg";
      if (u.getMass() != 1)
        stream << u.getMass();
      stream << ' ';
    }
    if (u.getLength()) {
      stream << 'm';
      if (u.getLength() != 1)
        stream << u.getLength();
      stream << ' ';
    }
    if (u.getTime()) {
      stream << 's';
      if (u.getTime() != 1)
        stream << u.getTime() << ' ';
      stream << ' ';
    }
    if (u.getTemperature()) {
      stream << 'K';
      if (u.getTemperature() != 1)
        stream << u.getTemperature();
      stream << ' ';
    }
    if (u.getCurrent()) {
      stream << 'A';
      if (u.getCurrent() != 1)
        stream << u.getCurrent();
      stream << ' ';
    }
    if (u.getLumIntensity()) {
      stream << "cd";
      if (u.getLumIntensity() != 1)
        stream << u.getLumIntensity();
      stream << ' ';
    }
  } else {
//     stream << "Unit: Factor = " << u.getFactor()
//          << ", Offset = " << u.getOffset()
//          << ", Length = " << u.getLength()
//          << ", Mass = " << u.getMass()
//          << ", Time = " << u.getTime()
//          << ", Temperature = " << u.getTemperature()
//          << ", Current = " << u.getCurrent()
//          << ", LumIntensity = " << u.getLumIntensity();


  }
  return stream;
}

// Returns a unit from the given string.
Unit getUnit(const std::string& unit);

// Convert from native units to the unit given in the unit argument.
real_type convertTo(Unit unit, real_type value);

// Convert to native units from the unit given in the unit argument.
real_type convertFrom(Unit unit, real_type value);

// Convert from native units to the unit given in the unit argument.
Vector3 convertTo(Unit unit, const Vector3& v);

// Convert to native units from the unit given in the unit argument.
Vector3 convertFrom(Unit unit, const Vector3& v);

} // namespace OpenFDM

#endif
