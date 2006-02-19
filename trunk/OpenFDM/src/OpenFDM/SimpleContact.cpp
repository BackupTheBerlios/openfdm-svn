/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Frame.h"
#include "Force.h"
#include "Contact.h"
#include "SimpleContact.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(SimpleContact, Contact)
  DEF_OPENFDM_PROPERTY(Real, SpringConstant, Serialized)
/// FIXME want to have similar names than with linearspringdamper
  DEF_OPENFDM_PROPERTY(Real, SpringDamping, Serialized)
  DEF_OPENFDM_PROPERTY(Real, FrictionCoeficient, Serialized)
  END_OPENFDM_OBJECT_DEF

SimpleContact::SimpleContact(const std::string& name) :
  Contact(name),
  mSpringConst(0),
  mSpringDamp(0),
  mFrictionCoef(0)
{
}

SimpleContact::~SimpleContact(void)
{
}

// Compute the plane normal force.
real_type
SimpleContact::computeNormalForce(real_type compressLen,
                                  real_type compressVel) const
{
  return compressLen*mSpringConst
    - mSpringDamp*min(compressVel, static_cast<real_type>(0));
}

// Compute the friction force.
Vector3
SimpleContact::computeFrictionForce(real_type normForce, const Vector3& vel,
                                    const Vector3&, real_type friction) const
{
  real_type nVel = norm(vel);
  if (1 < nVel)
    return (-friction*mFrictionCoef*normForce/nVel)*vel;
  else
    return (-friction*mFrictionCoef*normForce)*vel;
}

} // namespace OpenFDM
