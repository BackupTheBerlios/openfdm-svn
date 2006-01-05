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
  real_type soundspeed;
  real_type temperature;
};

class Atmosphere
  : public EnvironmentObject {
public:
  Atmosphere(void);
  virtual ~Atmosphere(void);

  // Get the atmosphere data for a given altitude alt.
  virtual AtmosphereData getData(real_type alt) const = 0;
};

} // namespace OpenFDM

#endif
