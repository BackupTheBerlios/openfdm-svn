/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Atmosphere.h"

namespace OpenFDM {

Atmosphere::Atmosphere(real_type gasConstant, real_type specificHeatRatio) :
  mGasConstant(gasConstant),
  mSpecificHeatRatio(specificHeatRatio)
{
}

Atmosphere::~Atmosphere(void)
{
}

} // namespace OpenFDM
