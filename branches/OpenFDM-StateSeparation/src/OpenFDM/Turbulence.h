/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Turbulence_H
#define OpenFDM_Turbulence_H

#include "Types.h"
#include "Object.h"
#include "Vector.h"

namespace OpenFDM {

/** Turbulence model.
 * Not yet something useful ...
 */

class Turbulence
  : public EnvironmentObject {
public:
  Turbulence(void) {}
  virtual ~Turbulence(void) {}

//   // Return the Turbulence velocity in the global coordinate frame.
//   virtual Vector3 getWindVel(const Vector3& pos) const /*= 0;*/
//   { return Vector3::zeros(); }
};

} // namespace OpenFDM

#endif
