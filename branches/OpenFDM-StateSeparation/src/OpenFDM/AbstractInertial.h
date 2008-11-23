/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_AbstractInertial_H
#define OpenFDM_AbstractInertial_H

#include "Referenced.h"
#include "Vector.h"

namespace OpenFDM {

class AbstractInertial : public Referenced {
public:
  virtual ~AbstractInertial();
  virtual Vector3 getAngularVelocity(const real_type& t) const;
  virtual Vector6 getAcceleration(const real_type& t) const;
};

} // namespace OpenFDM

#endif
