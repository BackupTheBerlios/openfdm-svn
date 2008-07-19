/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include <iostream>
#include <sstream>
#include <string>

#include <OpenFDM/Unit.h>

using namespace OpenFDM;

template<typename char_type, typename traits_type> 
inline
std::basic_istream<char_type, traits_type>&
operator>>(std::basic_istream<char_type, traits_type>& stream, Unit& u)
{
  // need to know the locale
  std::locale loc = stream.getloc();

  // eat whitespace
  stream >> std::ws;

  // a more or less hand written parser
  // since prefices are not distinguishable from units, this seems the
  // only alternative

  char_type first = stream.peek();
  if (first == stream.widen('c')) {
    stream.get();
    char_type second = stream.peek();
    if (second == stream.widen('a')) {
      stream.get();
      char_type third = stream.peek();
      if (third == stream.widen('l')) {
        // calories ...
        u = Unit::energy(4.1868/*FIXME*/);
        return stream;
      } else {
        stream.setstate(std::ios::failbit); // FIXME true?
        return stream;
      }
    } else if (second == stream.widen('m')) {
      // centimeters
      u = Unit::length(1e-2);
      return stream;
    } else {
      // now c must be centi = 1e-2
      Unit tmpU;
      stream >> tmpU;
      u = Unit(tmpU.getPhysicalDimension(), 1e-2*tmpU.getFactor(),
                tmpU.getOffset());
      return stream;
    }
  } else if (first == stream.widen('f')) {
    stream.get();
    char_type second = stream.peek();
    if (second == stream.widen('t')) {
      stream.get();
      // feet ...
      u = Unit::foot();
      return stream;
    } else {
      stream.setstate(std::ios::failbit); // FIXME true?
      return stream;
    }
  } else if (first == stream.widen('g')) {
    // g for gramm
    stream.get();
    u = Unit::mass(1e-3);
    return stream;
  } else if (first == stream.widen('h')) {
    // h
    stream.get();
    char_type second = stream.peek();
    if (second == stream.widen('o')) {
      // ho
      stream.get();
      char_type third = stream.peek();
      if (third == stream.widen('u')) {
        // hou
        stream.get();
        char_type fourth = stream.peek();
        if (fourth == stream.widen('r')) {
          // hour
          stream.get();
          u = Unit::hour();
          return stream;
        } else {
          // hou*
          stream.setstate(std::ios::failbit); // FIXME true?
          return stream;
        }
      } else {
        // ho*
        stream.setstate(std::ios::failbit); // FIXME true?
        return stream;
      }
    } else {
      // just h
      u = Unit::hour();
      return stream;
    }
  } else if (first == stream.widen('i')) {
    stream.get();
    char_type second = stream.peek();
    if (second == stream.widen('n')) {
      stream.get();
      // inch ...
      u = Unit::inch();
      return stream;
    } else {
      stream.setstate(std::ios::failbit); // FIXME true?
      return stream;
    }
  } else if (first == stream.widen('k')) {
    // k
    stream.get();
    char_type second = stream.peek();
    if (second == stream.widen('t')) {
      // kt
      stream.get();
      char_type third = stream.peek();
      if (third == stream.widen('s')) {
        // kts
        stream.get();
        u = Unit::knots();
        return stream;
      } else {
        stream.setstate(std::ios::failbit); // FIXME true?
        return stream;
      }
    } else {
      // k must be kilo = 1e3
      Unit tmpU;
      stream >> tmpU;
      u = Unit(tmpU.getPhysicalDimension(), 1e3*tmpU.getFactor(),
                tmpU.getOffset());
      return stream;
    }
  } else if (first == stream.widen('l')) {
    // l
    stream.get();
    char_type second = stream.peek();
    if (second == stream.widen('b')) {
      // lb
      stream.get();
      char_type third = stream.peek();
      if (third == stream.widen('s')) {
        // lbs
        stream.get();
        u = Unit::lbs();
        return stream;
      } else if (third == stream.widen('f')) {
        // lbf
        stream.get();
        u = Unit::lbf();
        return stream;
      } else {
        // just lb
        u = Unit::lbf();
        return stream;
      }
    } else {
      // just l is liters
      u = Unit::volume(1e-3);
      return stream;
    }
  } else if (first == stream.widen('m')) {
    // m
    stream.get();
    char_type second = stream.peek();
    if (second == stream.widen('m')) {
      // mm
      stream.get();
      // milimeter ...
      u = Unit::length(1e-3);
      return stream;
    } else if (second == stream.widen('i')) {
      // mi
      stream.get();
      char_type third = stream.peek();
      if (third == stream.widen('n')) {
        // min
        stream.get();
        u = Unit::minute();
        return stream;
      } else {
        stream.setstate(std::ios::failbit); // FIXME true?
        return stream;
      }
    } else if (std::isalpha(second, loc)) {
      // hmm m must be milli = 1e-3 ?? FIXME
      Unit tmpU;
      stream >> tmpU;
      u = Unit(tmpU.getPhysicalDimension(), 1e-3*tmpU.getFactor(),
                tmpU.getOffset());
      return stream;
    } else {
      // Ok, must have been a meters length
      u = Unit::length();
      return stream;
    }
  } else if (first == stream.widen('n')) {
    // n
    stream.get();
    char_type second = stream.peek();
    if (second == stream.widen('m')) {
      // nm
      stream.get();
      char_type third = stream.peek();
      if (third == stream.widen('i')) {
        // nmi for nautical miles
        stream.get();
        u = Unit::nauticalMile();
        return stream;
      } else {
        // just nm
        u = Unit::nauticalMile();
        return stream;
      }
    } else {
      stream.setstate(std::ios::failbit); // FIXME true?
      return stream;
    }
  } else if (first == stream.widen('s')) {
    // s
    stream.get();
    char_type second = stream.peek();
    if (second == stream.widen('e')) {
      // se
      stream.get();
      char_type third = stream.peek();
      if (third == stream.widen('c')) {
        // sec
        stream.get();
        u = Unit::time();
        return stream;
      } else {
        stream.setstate(std::ios::failbit); // FIXME true?
        return stream;
      }
    } else if (second == stream.widen('l')) {
      // sl
      stream.get();
      char_type third = stream.peek();
      if (third == stream.widen('u')) {
        // slu
        stream.get();
        char_type fourth = stream.peek();
        if (fourth == stream.widen('g')) {
          // slug
          stream.get();
          u = Unit::slug();
          return stream;
        } else {
          stream.setstate(std::ios::failbit); // FIXME true?
          return stream;
        }
      } else {
        stream.setstate(std::ios::failbit); // FIXME true?
        return stream;
      }
    } else if (second == stream.widen('m')) {
      // sm
      stream.get();
      char_type third = stream.peek();
      if (third == stream.widen('i')) {
        // smi for statute miles
        stream.get();
        u = Unit::statuteMile();
        return stream;
      } else {
        stream.setstate(std::ios::failbit); // FIXME true?
        return stream;
      }
    } else {
      // just s for seconds
      u = Unit::time();
      return stream;
    }
  } else if (first == stream.widen('C')) {
    // C elsius //FIXME C oulomb
    stream.get();
    u = Unit::degreeCelsius();
    return stream;
  } else if (first == stream.widen('F')) {
    // F arenheit //FIXME F arrad
    stream.get();
    u = Unit::degreeFarenheit();
    return stream;
  } else if (first == stream.widen('G')) {
    // G must be giga = 1e9
    stream.get();
    Unit tmpU;
    stream >> tmpU;
    u = Unit(tmpU.getPhysicalDimension(), 1e9*tmpU.getFactor(),
              tmpU.getOffset());
    return stream;
  } else if (first == stream.widen('J')) {
    // J oule
    stream.get();
    u = Unit::energy();
    return stream;
  } else if (first == stream.widen('K')) {
    // K elvin
    stream.get();
    u = Unit::temperature();
    return stream;
  } else if (first == stream.widen('M')) {
    // M must be mega = 1e6
    stream.get();
    Unit tmpU;
    stream >> tmpU;
    u = Unit(tmpU.getPhysicalDimension(), 1e6*tmpU.getFactor(),
              tmpU.getOffset());
    return stream;
  } else if (first == stream.widen('N')) {
    // N ewton
    stream.get();
    u = Unit::newton();
    return stream;
  } else if (first == stream.widen('P')) {
    // P
    stream.get();
    char_type second = stream.peek();
    if (second == stream.widen('a')) {
      // Pa
      stream.get();
      u = Unit::Pa();
      return stream;
    } else {
      stream.setstate(std::ios::failbit); // FIXME true?
      return stream;
    }
  } else if (first == stream.widen('R')) {
    // R ankine
    stream.get();
    u = Unit::rankine();
    return stream;
  } else if (first == stream.widen('T')) {
    // T must be terra = 1e12
    stream.get();
    Unit tmpU;
    stream >> tmpU;
    u = Unit(tmpU.getPhysicalDimension(), 1e12*tmpU.getFactor(),
              tmpU.getOffset());
    return stream;
  } else if (first == stream.widen('W')) {
    // W att
    stream.get();
    u = Unit::power();
    return stream;
  }



//   General
//      non-dimensional            ND              non-dimensional

//   Prefix 2
//      tera                       T               1e12 * parameter
//      giga                       G               1e9 * parameter
//      mega                       M               1e6 * parameter
//      kilo                       k               1e3 * parameter
//      deci                       d               1e-1 * parameter
//      centi                      c               1e-2 * parameter
//      milli                      m               1e-3 * parameter

//   Time
//      Solar year                 yr
//      Solar day                  day
//      hour                       h
//      minute                     min
//      second                     s               SI Standard

//   Length
//      metres                     m               SI Standard
//      inch                       in
//      feet                       ft
//      yard                       yd
//      nautical mile              nmi
//      statute mile               smi

//   Velocity 5
//      knot                       kn              nmi h-1

//   Volume
//      litre                      l               10-3 m3
//      US gallon                  USgal
//      UK gallon                  UKgal           Imperial Gallon

//   Mass
//      kilogram                   kg              SI Standard -- 103g
//      gram                       g
//      tonne                      tonne
//      slug                       slug
//      short ton                  USton           US Ton
//      long ton                   UKton           UK Ton
//      force at sealevel          lbs             not a mass as such, but handy

//   Force
//      Newton                     N               SI Standard
//      Pound Force                lbf

//   Pressure
//      Pascal                     Pa              N m-2
//      millimetres Mercury        mmHg
//      Pounds per square foot     lbf ft-2        lbf ft-2
//      Pounds per square inch     lbf in-2        lbf in-2
//      inches Mercury             inHg
//      Atmosphere                 atm

//   Temperature
//      degrees Kelvin             K               SI Standard
//      degrees Centigrade         C
//      degrees Farenheight        F
//      degrees Rankin             R

//   Viscosity
//      Poise                      Ps              Dynamic viscosity
//      Stokes                     St              Kinematic viscosity

//   Plane Angle
//      degree                     deg
//      radian                     rad
//      revolution                 rev

//   Power, Energy
//      Watt                       W               SI Standard
//      horsepower                 hp              550 ft lbf s-1
//      joule                      jou             SI Standard
//      British thermal unit       btu
//      calorie                    cal
//      erg                        erg

//   Electrical, Magnetic
//      volt direct current        Vdc
//      volt alternating current   Vac
//      ampere                     A               SI Standard
//      omh, resistance            ohm             SI Standard
//      cycle                      cyc
//      henry                      hy
//      farad                      fd
//      Telsa                      T               field strength
  return stream;
}

Unit stringToUnit(const std::string& s)
{
  std::stringstream sstr(s);
  Unit unit;
  sstr >> unit;
  return unit;
}

void printConversion(const std::string& s)
{
  std::cout << "Original String \"" << s << "\", parsed unit: \""
            << stringToUnit(s) << "\"" << std::endl;
}

int main(int argc, char *argv[])
{
  std::cout << "Mass units:\n";
  printConversion("kg");
  printConversion("mg");
  printConversion("g");
  printConversion("slug");
  printConversion("lbs");

  std::cout << "\nForce units:\n";
  printConversion("N");
  printConversion("kN");
  printConversion("mN");
  printConversion("lb");
  printConversion("lbf");

  std::cout << "\nLength units:\n";
  printConversion("m");
  printConversion("cm");
  printConversion("mm");
  printConversion("km");
  printConversion("nmi");
  printConversion("nm");
  printConversion("smi");
  printConversion("ft");
  printConversion("in");
  printConversion("inch");

  std::cout << "\nSpeed units:\n";
  printConversion("kts"); // FIXME
  printConversion("m/s"); // FIXME
  printConversion("m s-1"); // FIXME
  printConversion("km h-1"); // FIXME
  printConversion("km/h"); // FIXME
  printConversion("ft/s"); // FIXME
  printConversion("ft s-1"); // FIXME
  printConversion("ft/min"); // FIXME
  printConversion("ft min-1"); // FIXME

  std::cout << "\nAngular nonunits:\n";
  printConversion("deg");
  printConversion("rad");

  std::cout << "\nTime units:\n";
  printConversion("s");
  printConversion("sec");
  printConversion("min");
  printConversion("h");
  printConversion("hour");

  std::cout << "\nTemperature units:\n";
  printConversion("K");
  printConversion("R");
  printConversion("C");
  printConversion("F");

  std::cout << "\nPressure units:\n";
  printConversion("Pa");

  std::cout << "\nEnergy units:\n";
  printConversion("Nm"); // FIXME
  printConversion("J");
  printConversion("jou"); // FIXME
  printConversion("cal");

  std::cout << "\nPower units:\n";
  printConversion("W");

//   std::cout << "m/s:       " << Unit::meterPerSecond << std::endl;
//   std::cout << "km/h:      " << Unit::kiloMeterPerHour << std::endl;
//   std::cout << "ft/s:      " << Unit::feetPerSecond << std::endl;
//   std::cout << "ft/min:    " << Unit::feetPerMinute << std::endl;
//   std::cout << "kts:       " << Unit::knots << std::endl;

//   std::cout << "m/s^2:     " << Unit::meterPerSecond2 << std::endl;

  return 0;
}

