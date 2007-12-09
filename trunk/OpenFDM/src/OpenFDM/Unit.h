/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich
 *
 */

#ifndef OpenFDM_Unit_H
#define OpenFDM_Unit_H

#include <string>

#include "LogStream.h"
#include "PhysicalDimension.h"
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

class Unit;

inline Unit operator*(const Unit& u1, const Unit& u2);
inline Unit operator/(const Unit& u1, const Unit& u2);
inline Unit exp(const Unit& u, signed char exponent);

class Unit {
public:
  Unit(void) :
    mFactor(1),
    mOffset(0)
  { }
  Unit(real_type factor) :
    mFactor(factor),
    mOffset(0)
  { }
  Unit(const PhysicalDimension& physDim) :
    mPhysicalDimension(physDim),
    mFactor(1),
    mOffset(0)
  { }
  Unit(const PhysicalDimension& physDim, real_type factor) :
    mPhysicalDimension(physDim),
    mFactor(factor),
    mOffset(0)
  { }
  Unit(const PhysicalDimension& physDim, real_type factor, real_type offset) :
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

  static Unit dimless()
  { return Unit(); }

  static Unit radian()
  { return Unit(); }
  static Unit degree()
  { return Unit(deg2rad); }

  static Unit second()
  { return time(); }
  static Unit minute()
  { return time(60); }
  static Unit hour()
  { return time(60*60); }
  static Unit day()
  { return time(60*60*24); }
  static Unit week()
  { return time(60*60*24*7); }

  static Unit meter()
  { return length(); }
  static Unit kilometer()
  { return length(1000); }
  static Unit foot()
  { return length(real_type(0.3048)); }
  static Unit inch()
  { return length(real_type(0.3048)/12); }
  static Unit nauticalMile()
  { return length(1852); }
  static Unit statuteMile()
  { return length(5280*real_type(0.3048)); }

  static Unit meterPerSecond()
  { return velocity(); }
  static Unit kiloMeterPerHour()
  { return kilometer()/hour(); }
  static Unit footPerSecond()
  { return foot()/second(); }
  static Unit footPerMinute()
  { return foot()/minute(); }
  static Unit knots()
  { return nauticalMile()/hour(); }

  static Unit meterPerSecond2()
  { return meter()/exp(second(), 2); }
  static Unit footPerSecond2()
  { return foot()/exp(second(), 2); }

  static Unit squareMeter()
  { return exp(meter(), 2); }
  static Unit squareFoot()
  { return exp(foot(), 2); }

  static Unit qubicMeter()
  { return exp(meter(), 3); }
  static Unit qubicFoot()
  { return exp(foot(), 3); }

  static Unit kilogramm()
  { return mass(); }
  static Unit lbs()
  { return mass(1/real_type(2.20462262184877566540)); }
  static Unit slug()
  { return mass(1/real_type(6.85217658567917470291e-2)); }

  static Unit kilogrammSquareMeter()
  { return kilogramm()*squareMeter(); }
  static Unit slugSquareFoot()
  { return slug()*squareFoot(); }

  static Unit newton()
  { return force(); }
  static Unit lbf()
  { return slug()*footPerSecond2(); }

  static Unit newtonMeter()
  { return newton()*meter(); }
  static Unit lbfFoot()
  { return lbf()*foot(); }
  
  static Unit newtonPerMeter()
  { return force()/length(); }
  static Unit lbfPerFoot()
  { return lbf()/foot(); }

  static Unit newtonPerMeterPerSecond()
  { return newton()/velocity(); }
  static Unit lbfPerFootPerSecond()
  { return lbf()/footPerSecond(); }

  static Unit Pa()
  { return force()/area(); }
  static Unit lbfPerSquareFoot()
  { return lbf()/squareFoot(); }

  static Unit kelvin()
  { return temperature(); }
  static Unit degreeCelsius()
  { return temperature(1, real_type(273.15)); }
  static Unit rankine()
  { return temperature(real_type(5)/real_type(9)); }
  static Unit degreeFarenheit()
  {
    return temperature(real_type(5)/real_type(9),
                       real_type(273.15) - real_type(32*5)/real_type(9));
  }

  static Unit length(real_type factor = 1)
  { return Unit(PhysicalDimension::length(), factor); }
  static Unit mass(real_type factor = 1)
  { return Unit(PhysicalDimension::mass(), factor); }
  static Unit time(real_type factor = 1)
  { return Unit(PhysicalDimension::time(), factor); }
  static Unit temperature(real_type factor = 1, real_type offset = 0)
  { return Unit(PhysicalDimension::temperature(), factor, offset); }
  static Unit current(real_type factor = 1)
  { return Unit(PhysicalDimension::current(), factor); }
  static Unit amountSubstance(real_type factor = 1)
  { return Unit(PhysicalDimension::amountSubstance(), factor); }
  static Unit lumIntensity(real_type factor = 1)
  { return Unit(PhysicalDimension::lumIntensity(), factor); }
  static Unit area(real_type factor = 1)
  { return Unit(PhysicalDimension::area(), factor); }
  static Unit volume(real_type factor = 1)
  { return Unit(PhysicalDimension::volume(), factor); }
  static Unit velocity(real_type factor = 1)
  { return length(factor)/time(); }
  static Unit acceleration(real_type factor = 1)
  { return velocity(factor)/time(); }
  static Unit jerk(real_type factor = 1)
  { return acceleration(factor)/time(); }
  static Unit force(real_type factor = 1)
  { return mass(factor)*acceleration(); }
  static Unit pressure(real_type factor = 1)
  { return force(factor)/area(); }
  static Unit density(real_type factor = 1)
  { return mass(factor)/volume(); }
  static Unit energy(real_type factor = 1)
  { return force(factor)*length(); }
  static Unit power(real_type factor = 1)
  { return energy(factor)/time(); }
  static Unit inertia(real_type factor = 1)
  { return mass(factor)*area(); }

  // Convert from native units to the unit given in the unit argument.
  real_type convertTo(const real_type& value) const
  { return (value - mOffset)/mFactor; }
  // Convert to native units from the unit given in the unit argument.
  real_type convertFrom(const real_type& value) const
  { return mFactor*value + mOffset; }

  // Convert from native units to the unit given in the unit argument.
  template<typename Impl, LinAlg::size_type m, LinAlg::size_type n>
  LinAlg::Matrix<typename Impl::value_type,m,n>
  convertTo(const LinAlg::MatrixRValue<Impl,m,n>& A) const
  {
    const Impl& Ai = A.asImpl();

    LinAlg::size_type rows = Ai.rows();
    LinAlg::size_type cols = Ai.cols();
    
    LinAlg::Matrix<typename Impl::value_type,m,n> ret(rows, cols);
    for (LinAlg::size_type j = 0; j < cols; ++j)
      for (LinAlg::size_type i = 0; i < rows; ++i)
        ret(i,j) = convertTo(Ai(i,j));
    
    return ret;
  }
  // Convert to native units from the unit given in the unit argument.
  template<typename Impl, LinAlg::size_type m, LinAlg::size_type n>
  LinAlg::Matrix<typename Impl::value_type,m,n>
  convertFrom(const LinAlg::MatrixRValue<Impl,m,n>& A) const
  {
    const Impl& Ai = A.asImpl();

    LinAlg::size_type rows = Ai.rows();
    LinAlg::size_type cols = Ai.cols();
    
    LinAlg::Matrix<typename Impl::value_type,m,n> ret(rows, cols);
    for (LinAlg::size_type j = 0; j < cols; ++j)
      for (LinAlg::size_type i = 0; i < rows; ++i)
        ret(i,j) = convertFrom(Ai(i,j));
    
    return ret;
  }

private:
  real_type mFactor;
  real_type mOffset;
  PhysicalDimension mPhysicalDimension;
};

inline
Unit
operator*(const Unit& u1, const Unit& u2)
{
  return Unit(u1.getPhysicalDimension()*u2.getPhysicalDimension(),
              u1.getFactor()*u2.getFactor(),
              u1.getOffset()+u1.getFactor()*u2.getOffset());
}

inline
Unit
operator/(const Unit& u1, const Unit& u2)
{
  return Unit(u1.getPhysicalDimension()/u2.getPhysicalDimension(),
              u1.getFactor()/u2.getFactor(),
              u1.getOffset()/u2.getFactor()-u2.getOffset());
}

inline
Unit
exp(const Unit& u, signed char exponent)
{
  if (exponent == 0)
    return Unit();
  else if (exponent == 1)
    return u;
  else if (1 < exponent)
    return u*exp(u, exponent-1);
  else if (exponent < 0)
    return Unit()/exp(u, -exponent);
}

inline
bool
operator==(const Unit& u1, const Unit& u2)
{
  return u1.getFactor() == u2.getFactor() &&
    u1.getOffset() == u2.getOffset() &&
    u1.getPhysicalDimension() == u2.getPhysicalDimension();
}

inline
bool
operator!=(const Unit& u1, const Unit& u2)
{
  return u1.getFactor() != u2.getFactor() ||
    u1.getOffset() != u2.getOffset() ||
    u1.getPhysicalDimension() != u2.getPhysicalDimension();
}

inline
bool
compatible(const Unit& u1, const Unit& u2)
{
  return u1.getPhysicalDimension() == u2.getPhysicalDimension();
}

template<typename char_type, typename traits_type> 
inline
std::basic_ostream<char_type, traits_type>&
operator<<(std::basic_ostream<char_type, traits_type>& stream, const Unit& u)
{
  stream << "Unit: Factor = " << u.getFactor()
         << ", Offset = " << u.getOffset()
         << stream.widen(' ');

  // Distinguish between offset free and units with offset.
  PhysicalDimension physicalDimension = u.getPhysicalDimension();
  if (u.getOffset() == 0) {
    if (physicalDimension.getMass()) {
      stream << stream.widen('k') << stream.widen('g');
      if (physicalDimension.getMass() != 1)
        stream << int(physicalDimension.getMass());
      stream << stream.widen(' ');
    }
    if (physicalDimension.getLength()) {
      stream << stream.widen('m');
      if (physicalDimension.getLength() != 1)
        stream << int(physicalDimension.getLength());
      stream << stream.widen(' ');
    }
    if (physicalDimension.getTime()) {
      stream << stream.widen('s');
      if (physicalDimension.getTime() != 1)
        stream << int(physicalDimension.getTime()) << stream.widen(' ');
      stream << stream.widen(' ');
    }
    if (physicalDimension.getTemperature()) {
      stream << stream.widen('K');
      if (physicalDimension.getTemperature() != 1)
        stream << int(physicalDimension.getTemperature());
      stream << stream.widen(' ');
    }
    if (physicalDimension.getCurrent()) {
      stream << stream.widen('A');
      if (physicalDimension.getCurrent() != 1)
        stream << int(physicalDimension.getCurrent());
      stream << stream.widen(' ');
    }
    if (physicalDimension.getLumIntensity()) {
      stream << stream.widen('c') << stream.widen('d');
      if (physicalDimension.getLumIntensity() != 1)
        stream << int(physicalDimension.getLumIntensity());
      stream << stream.widen(' ');
    }
  } else {
    stream << ", Length = " << int(physicalDimension.getLength())
           << ", Mass = " << int(physicalDimension.getMass())
           << ", Time = " << int(physicalDimension.getTime())
           << ", Temperature = " << int(physicalDimension.getTemperature())
           << ", Current = " << int(physicalDimension.getCurrent())
           << ", LumIntensity = " << int(physicalDimension.getLumIntensity());
  }
  return stream;
}

} // namespace OpenFDM

#endif
