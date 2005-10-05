/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Types.h"
#include "Math.h"
#include "Vector.h"
#include "Units.h"

namespace OpenFDM {

#ifdef PI
#undef PI
#endif
#define PI static_cast<real_type>(3.1415926535897932384626433832795029L)

const real_type pi = PI;
const real_type pi2 = static_cast<real_type>(0.5L)*PI;
const real_type pi4 = static_cast<real_type>(0.25L)*PI;

const real_type deg2rad = PI/static_cast<real_type>(180);
const real_type rad2deg = static_cast<real_type>(180)/PI;

// The newtonian gravity constant.
const real_type gravity_constant = 6.673e-11;

struct UnitNames {
  Unit unit;
  const char* names[31];
};

static UnitNames unit_names[] = {
  { uRadian,          { "rad", 0 } },
  { uDegree,          { "deg", "degree", "degrees", 0 } },
  { uSecond,          { "s", "sec", "second", "seconds", 0 } },
  { uMinute,          { "min", "minute", "minutes", 0 } },
  { uHour,            { "h", "hour", "hours", 0 } },
  { uMeter,           { "m", "meter", 0 } },
  { uFoot,            { "ft", "foot", "feet", 0 } },
  { uInch,            { "in", "inch", 0 } },
  { uMile,            { "mile", 0 } },
  { uNauticalMile,    { "nm", 0 } },
  { uMeterPSecond,    { "m/s", "meter/s", 0 } },
  { uKiloMeterPHour,  { "km/h", 0 } },
  { uFeetPSecond,     { "ft/s", 0 } },
  { uFeetPMinute,     { "ft/m", 0 } },
  { uKnots,           { "knots", "kts", "kt", 0 } },
  { uMeterPSec2,      { "m/s2", 0 } },
  { uFeetPSec2,       { "ft/s2", 0 } },
  { uNewton,          { "N", "newton", 0 } },
  { uPoundForce,      { "lbf", "pound", 0 } },
  { uKilogram,        { "kg", 0 } },
  { uPoundSealevel,   { "lbs", 0 } },
  { uSlug,            { "slug", 0 } },
  { uKilogramMeter2,  { "kgm2", "kg-m2", 0 } },
  { uSlugFt2,         { "slugft2", "slug-ft2", 0 } },
  { uKelvin,          { "K", "kelvin", 0 } },
  { uDegC,            { "degC", "C", 0 } },
  { uRankine,         { "rankine", 0 } },
  { uFahrenheit,      { "fahrenheit", "F", 0 } },
  { uNewtonPMeter,    { "N/m", 0 } },
  { uPoundForcePFt,   { "lbf/ft", 0 } },
  { uPascal,          { "pascal", "pa", "N/m2", 0 } },
  { uPoundPFt2,       { "lbf/ft2", 0 } },
  { uKilogramPMeter3, { "kg/m3", 0 } },
  { uSlugPFt3,        { "slug/ft3", 0 } },
  { uUnknown,         { 0 } }
};

// Returns a unit from the given string.
Unit getUnit(const std::string& unit)
{
  // Walk through all names and return the unit type if it is found.
  int i = 0;
  while (unit_names[i].unit != uUnknown) {
    int j = 0;
    while (unit_names[i].names[j]) {
      if (unit_names[i].names[j] == unit)
        return unit_names[i].unit;
      ++j;
    }
    ++i;
  }
  return uUnknown;
}

#define ONE static_cast<real_type>(1)

// Taken from the units UNIX commandline tool
#define FOOT2METER static_cast<real_type>(0.3048)
#define FOOT2INCH static_cast<real_type>(12)
#define MILE2FOOT static_cast<real_type>(5280)
#define NAUTICALMILE2METER static_cast<real_type>(1852)
#define KILOGRAM2SLUG static_cast<real_type>(6.85217658567917470291e-2)

// Could that be derived???
#define KILOGRAM2POUND static_cast<real_type>(2.20462262184877566540)

// Derived but handy defines
#define NEWTON2POUND (KILOGRAM2SLUG/FOOT2METER)

// convert from native units to the unit given in the unit argument.
real_type convertTo(Unit unit, real_type value)
{
  switch (unit) {
    // Angle 'unit'
  case uRadian:
    return value;
  case uDegree:
    return value*rad2deg;

    // Time unit
  case uSecond:
    return value;
  case uMinute:
    return value*(ONE/static_cast<real_type>(60));
  case uHour:
    return value*(ONE/static_cast<real_type>(3600));

    // Length units.
  case uMeter:
    return value;
  case uFoot:
    return value*(ONE/FOOT2METER);
  case uInch:
    return value*(FOOT2INCH/FOOT2METER);
  case uMile:
    return value*(ONE/(FOOT2METER*MILE2FOOT));
  case uNauticalMile:
    return value*(ONE/NAUTICALMILE2METER);

    // Area units.
  case uMeter2:
    return value;
  case uFoot2:
    return value*(ONE/FOOT2METER)*(ONE/FOOT2METER);

    // Speed units.
  case uMeterPSecond:
    return value;
  case uKiloMeterPHour:
    return value*(static_cast<real_type>(36)/static_cast<real_type>(10));
  case uFeetPSecond:
    return value*(ONE/FOOT2METER);
  case uFeetPMinute:
    return value*(static_cast<real_type>(60)/FOOT2METER);
  case uKnots:
    return value*(static_cast<real_type>(3600)/NAUTICALMILE2METER);

    // Acceleration units.
  case uMeterPSec2:
    return value;
  case uFeetPSec2:
    return value*(ONE/FOOT2METER);

    // Force units.
  case uNewton:
    return value;
  case uPoundForce:
    return value*NEWTON2POUND;

    // Energy units (moments).
  case uNewtonMeter:
    return value;
  case uPoundForceFt:
    return value*(NEWTON2POUND/FOOT2METER);

    // Mass units.
  case uKilogram:
    return value;
  case uPoundSealevel:
    return value*KILOGRAM2POUND;
  case uSlug:
    return value*KILOGRAM2SLUG;

    // Inertia units.
  case uKilogramMeter2:
    return value;
  case uSlugFt2:
    return value*(KILOGRAM2SLUG/(FOOT2METER*FOOT2METER));

    // Temperature units.
  case uKelvin:
    return value;
  case uDegC:
    return value - static_cast<real_type>(273.15);
  case uRankine:
    return value*(static_cast<real_type>(5)/static_cast<real_type>(9));
  case uFahrenheit:
    return value*(static_cast<real_type>(5)/static_cast<real_type>(9))
      - static_cast<real_type>(5*27315)/static_cast<real_type>(900)
      + static_cast<real_type>(32);

    // Spring stiffness.
  case uNewtonPMeter:
    return value;
  case uPoundForcePFt:
    return value*(NEWTON2POUND*FOOT2METER);
  case uPoundForcePInch:
    return value*(NEWTON2POUND*FOOT2METER/FOOT2INCH);

    // Pressure units.
  case uPascal:
    return value;
  case uPoundPFt2:
    return value*(NEWTON2POUND*FOOT2METER*FOOT2METER);

    // Density units.
  case uKilogramPMeter3:
    return value;
  case uSlugPFt3:
    return value*(KILOGRAM2SLUG*FOOT2METER*FOOT2METER*FOOT2METER);

    // Hmm ...
  default:
    return static_cast<real_type>(0);
  }
}

// convert to native units from the unit given in the unit argument.
real_type convertFrom(Unit unit, real_type value)
{
  switch (unit) {
    // Angle 'unit'
  case uRadian:
    return value;
  case uDegree:
    return value*deg2rad;
    
    // Time unit
  case uSecond:
    return value;
  case uMinute:
    return value*static_cast<real_type>(60);
  case uHour:
    return value*static_cast<real_type>(3600);

    // Length units.
  case uMeter:
    return value;
  case uFoot:
    return value*FOOT2METER;
  case uInch:
    return value*(FOOT2METER/FOOT2INCH);
  case uMile:
    return value*(FOOT2METER*MILE2FOOT);
  case uNauticalMile:
    return value*NAUTICALMILE2METER;

    // Area units.
  case uMeter2:
    return value;
  case uFoot2:
    return value*FOOT2METER*FOOT2METER;
    
    // Speed units.
  case uMeterPSecond:
    return value;
  case uKiloMeterPHour:
    return value*(static_cast<real_type>(10)/static_cast<real_type>(36));
  case uFeetPSecond:
    return value*FOOT2METER;
  case uFeetPMinute:
    return value*(FOOT2METER/static_cast<real_type>(60));
  case uKnots:
    return value*(NAUTICALMILE2METER/static_cast<real_type>(3600));
    
    // Acceleration units.
  case uMeterPSec2:
    return value;
  case uFeetPSec2:
    return value*FOOT2METER;
    
    // Force units.
  case uNewton:
    return value;
  case uPoundForce:
    return value*(ONE/NEWTON2POUND);
    
    // Energy units (moments).
  case uNewtonMeter:
    return value;
  case uPoundForceFt:
    return value*(FOOT2METER/NEWTON2POUND);

    // Mass units.
  case uKilogram:
    return value;
  case uPoundSealevel:
    return value*(ONE/KILOGRAM2POUND);
  case uSlug:
    return value*(ONE/KILOGRAM2SLUG);
    
    // Inertia units.
  case uKilogramMeter2:
    return value;
  case uSlugFt2:
    return value*(FOOT2METER*FOOT2METER/KILOGRAM2SLUG);
    
    // Temperature units.
  case uKelvin:
    return value;
  case uDegC:
    return value + static_cast<real_type>(273.15);
  case uRankine:
    return value*(static_cast<real_type>(9)/static_cast<real_type>(5));
  case uFahrenheit:
    return value*(static_cast<real_type>(9)/static_cast<real_type>(5))
      + static_cast<real_type>(273.15)
      - static_cast<real_type>(32*9)/static_cast<real_type>(5);

    // Spring stiffness.
  case uNewtonPMeter:
    return value;
  case uPoundForcePFt:
    return value*(ONE/(NEWTON2POUND*FOOT2METER));
  case uPoundForcePInch:
    return value*(FOOT2INCH/(NEWTON2POUND*FOOT2METER));

    // Pressure units.
  case uPascal:
    return value;
  case uPoundPFt2:
    return value*(ONE/(NEWTON2POUND*FOOT2METER*FOOT2METER));

    // Density units.
  case uKilogramPMeter3:
    return value;
  case uSlugPFt3:
    return value*(ONE/(KILOGRAM2SLUG*FOOT2METER*FOOT2METER*FOOT2METER));

    // Hmm ...
  default:
    return static_cast<real_type>(0);
  }
}

// Convert from native units to the unit given in the unit argument.
Vector3 convertTo(Unit unit, const Vector3& v)
{
  return Vector3(convertTo(unit, v(1)),
                 convertTo(unit, v(2)),
                 convertTo(unit, v(3)));
}

// Convert to native units from the unit given in the unit argument.
Vector3 convertFrom(Unit unit, const Vector3& v)
{
  return Vector3(convertFrom(unit, v(1)),
                 convertFrom(unit, v(2)),
                 convertFrom(unit, v(3)));
}

Unit2 Unit2::dimless;

Unit2 Unit2::radian;
Unit2 Unit2::degree(PI/(real_type)180);

Unit2 Unit2::second(Unit2::time());
Unit2 Unit2::minute(Unit2::time(60));
Unit2 Unit2::hour(Unit2::time(60*60));
Unit2 Unit2::day(Unit2::time(60*60*24));
Unit2 Unit2::week(Unit2::time(60*60*24*7));

Unit2 Unit2::meter(Unit2::length());
Unit2 Unit2::kilometer(Unit2::length(1000));
Unit2 Unit2::feet(Unit2::length(FOOT2METER));
Unit2 Unit2::inch(Unit2::length(FOOT2METER/FOOT2INCH));
Unit2 Unit2::nauticalMile(Unit2::length(NAUTICALMILE2METER));
Unit2 Unit2::statuteMile(Unit2::length(MILE2FOOT*FOOT2METER));

Unit2 Unit2::meterPerSecond(Unit2::velocity());
Unit2 Unit2::kiloMeterPerHour(Unit2::length(1000)/Unit2::time(60*60));
Unit2 Unit2::feetPerSecond(Unit2::velocity(FOOT2METER));
Unit2 Unit2::feetPerMinute(Unit2::length(FOOT2METER)/Unit2::time(60));
Unit2 Unit2::knots(Unit2::length(NAUTICALMILE2METER)/Unit2::time(60*60));

Unit2 Unit2::meterPerSecond2(Unit2::acceleration());

Unit2 Unit2::kilogramm(Unit2::mass());
Unit2 Unit2::poundMass(Unit2::mass(ONE/KILOGRAM2POUND));
Unit2 Unit2::lbs(Unit2::mass(ONE/KILOGRAM2POUND));
Unit2 Unit2::slugs(Unit2::mass(ONE/KILOGRAM2SLUG));

Unit2 Unit2::N(Unit2::force());
Unit2 Unit2::poundForce(Unit2::force(ONE/NEWTON2POUND));
Unit2 Unit2::lbf(Unit2::force(ONE/NEWTON2POUND));

Unit2 Unit2::Pa(Unit2::force()/Unit2::area());

Unit2 Unit2::kelvin(Unit2::temperature());
Unit2 Unit2::degreeCelsius(Unit2::temperature(1, (real_type)-273.15));
Unit2 Unit2::rankine(Unit2::temperature(9/(real_type)5));
Unit2 Unit2::degreeFarenheit(Unit2::temperature(9/(real_type)5, -273.15*9/(real_type)5));

} // namespace OpenFDM

