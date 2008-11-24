/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "AbstractAtmosphere.h"

namespace OpenFDM {

AbstractAtmosphere::AbstractAtmosphere(const real_type& gasConstant) :
  mGasConstant(gasConstant)
{
}

AbstractAtmosphere::~AbstractAtmosphere(void)
{
}

AtmosphereData
AbstractAtmosphere::getData(const real_type& alt) const
{
  AtmosphereData data;
  // Sea level pressure = 101325 N/m2
  data.pressure = 101325;
  // Sea level temperature = 288.15 K
  data.temperature = 288.15;
  // Sea leve density of 1.225 kg/m3.
  data.density = 1.225;
  return data;
}

} // namespace OpenFDM
