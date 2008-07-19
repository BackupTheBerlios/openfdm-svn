/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Atmosphere.h"

namespace OpenFDM {

Atmosphere::Atmosphere(real_type gasConstant) :
  mGasConstant(gasConstant)
{
}

Atmosphere::~Atmosphere(void)
{
}

} // namespace OpenFDM
