/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_SimpleContact_H
#define OpenFDM_SimpleContact_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Frame.h"
#include "Force.h"
#include "Contact.h"

namespace OpenFDM {

class SimpleContact : public Contact {
  OPENFDM_OBJECT(SimpleContact, Contact);
public:
  SimpleContact(const std::string& name);
  virtual ~SimpleContact(void);

  const real_type& getSpringConstant(void) const
  { return mSpringConst; }
  void setSpringConstant(const real_type& springConst)
  { mSpringConst = springConst; }

  const real_type& getSpringDamping(void) const
  { return mSpringDamp; }
  void setSpringDamping(const real_type& springDamp)
  { mSpringDamp = springDamp; }

  const real_type& getFrictionCoeficient(void) const
  { return mFrictionCoef; }
  void setFrictionCoeficient(const real_type& frictionCoef)
  { mFrictionCoef = frictionCoef; }

  // Compute the plane normal force.
  virtual real_type
  computeNormalForce(real_type compressLen, real_type compressVel) const;

  // Compute the friction force.
  virtual Vector3
  computeFrictionForce(real_type normForce, const Vector3& vel,
                       const Vector3&, real_type friction) const;

private:
  real_type mSpringConst;
  real_type mSpringDamp;
  real_type mFrictionCoef;
};

} // namespace OpenFDM

#endif
