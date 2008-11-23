/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_AbstractGravity_H
#define OpenFDM_AbstractGravity_H

#include "Referenced.h"
#include "Vector.h"

namespace OpenFDM {

class Environment;

class AbstractGravity : public Referenced {
public:
  virtual ~AbstractGravity();
  virtual Vector3 getGravityAcceleration(const Environment&,
                                         const Vector3&) const;
};

} // namespace OpenFDM

#endif
