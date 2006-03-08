/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include <iterator>

#include "Types.h"
#include "LogStream.h"
#include "Math.h"
#include "Object.h"
#include "Atmosphere.h"
#include "AtmosphereSTD1976.h"
#include "Environment.h"

#include <iostream>

namespace OpenFDM {

// Hydrostatic constant = 34.1631947 kelvin/km
const real_type AtmosphereSTD1976::mHydrostaticConstant = 34.1631947;

// Radius of the Earth (km)
const real_type AtmosphereSTD1976::mEarthRadius = 6369.0;

#define MOL_WT          28.9644  // kg/kgmol (air)
#define R_HAT           8314.32  // J/kgmol.K (gas const.)

AtmosphereSTD1976::AtmosphereSTD1976(void) :
  Atmosphere(R_HAT/MOL_WT)
{
  mTable[0.0] = TableData(288.15, 1.0, -6.5);
  mTable[11.0] = TableData(216.65, 2.233611e-1, 0.0);
  mTable[20.0] = TableData(216.65, 5.403295e-2, 1.0);
  mTable[32.0] = TableData(228.65, 8.5666784e-3, 2.8);
  mTable[47.0] = TableData(270.65, 1.0945601e-3, 0.0);
  mTable[51.0] = TableData(270.65, 6.6063531e-4, -2.8);
  mTable[71.0] = TableData(214.65, 3.9046834e-5, -2.0);
  mTable[84.852] = TableData(186.946, 3.68501e-6, 0.0);
}

AtmosphereSTD1976::~AtmosphereSTD1976(void)
{
}

AtmosphereData
AtmosphereSTD1976::getData(real_type alt) const
{
  // We cannot compute data for negative altitudes, just treat them as 0 alt
  if (alt < 0.0)
    alt = 0.0;

  // Convert altitude to km
  alt *= 1e-3;

  // Convert geometric to geopotential altitude
  real_type h = alt*mEarthRadius/(alt+mEarthRadius);
  
  // Lookup in the table for the required values.
  Table::const_iterator it0 = mTable.begin();
  Table::const_iterator firstBelow = --mTable.upper_bound(h);
  if (firstBelow == mTable.end())
    firstBelow == it0;

  real_type tgrad = firstBelow->second.g;
  real_type tbase = firstBelow->second.t;
  real_type deltah = h - firstBelow->first;
  real_type tlocal = tbase + tgrad*deltah;

  // temperature ratio
  real_type theta = tlocal/(*it0).second.t;

  real_type delta, sigma;

  // pressure ratio
  if (tgrad == 0.0)
    delta = firstBelow->second.p*exp(-mHydrostaticConstant*deltah/tbase);
  else
    delta = firstBelow->second.p*pow(tbase/tlocal, mHydrostaticConstant/tgrad);

  // density ratio
  sigma = delta/theta;


  // Sea level pressure = 101325 N/m2
  real_type slPressure = 101325;
  // Sea level temperature = 288.15 K
  real_type slTemperature = 288.15;

  // Sea leve density of 1.225 kg/m3.
  real_type slDensity = 1.225;

  AtmosphereData data;

  // Is aequivalent to one bar.
  data.pressure = slPressure*delta;
  // Temperature in kelvin
  data.temperature = slTemperature*theta;

  if (fabs(data.temperature) > Limits<real_type>::min())
    data.density = data.pressure / (getGasConstant()*data.temperature);
  else
    data.density = 0.0;

  Log(Environment, Debug) << "p = " << data.pressure << ", T = "
                          << data.temperature << ", rho = "
                          << data.density << endl;
  
  return data;
}

} // namespace OpenFDM
