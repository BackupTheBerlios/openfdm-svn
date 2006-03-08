/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Atmosphere_H
#define OpenFDM_Atmosphere_H

#include "Types.h"
#include "EnvironmentObject.h"

namespace OpenFDM {

struct AtmosphereData {
  real_type pressure;
  real_type density;
  real_type temperature;
};

class Atmosphere
  : public EnvironmentObject {
public:
  virtual ~Atmosphere(void);

  /// Returns the gas constant for that current atmosphere gas often refered
  /// as R. The unit is as usual in the SI system m^2/(s^2 K)
  real_type getGasConstant(void) const
  { return mGasConstant; }
  real_type getR(void) const
  { return mGasConstant; }

  /// Returns the soundspeed for the given temperature
  real_type getSoundSpeed(real_type temperature) const
  { return sqrt(getGamma(temperature)*temperature*getGasConstant()); }

  /// Returns the specific heat ratio
  real_type getSpecificHeatRatio(real_type temperature) const
  { return getGamma(temperature); }
  real_type getGamma(real_type temperature) const
  {
    // Taken from SimTurbine, converted to kelvin
    real_type a = -4.48729540632e-12;
    real_type b =  4.459750164e-08;
    real_type c = -1.407342762e-04;
    real_type d =  1.436914;
    return ((a*temperature + b)*temperature + c)*temperature + d;
  }

  /// Returns the specific heat constant
  real_type getCp(real_type temperature) const
  {
    real_type gamma = getGamma(temperature);
    return getGasConstant()*gamma/(gamma-1);
  }

  // Get the atmosphere data for a given altitude alt.
  virtual AtmosphereData getData(real_type alt) const = 0;

protected:
  Atmosphere(real_type gasConstant);

private:
  real_type mGasConstant;
};

} // namespace OpenFDM

#endif
