/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Wind_H
#define OpenFDM_Wind_H

#include "Types.h"
#include "EnvironmentObject.h"
#include "Vector.h"

namespace OpenFDM {

class Wind
  : public EnvironmentObject {
public:
  Wind(void) {}
  virtual ~Wind(void) {}

  // Return the wind velocity in the global coordinate frame.
  // ????
  // FIXME: make pure virtual
  virtual Vector3 getWindVel(const Vector3& pos) const /*= 0;*/
  { return Vector3::zeros(); }
};

} // namespace OpenFDM

#endif
