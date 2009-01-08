/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "SimpleContact.h"

#include "Assert.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(SimpleContact, Contact)
  DEF_OPENFDM_PROPERTY(Real, SpringConstant, Serialized)
  DEF_OPENFDM_PROPERTY(Real, DamperConstant, Serialized)
  DEF_OPENFDM_PROPERTY(Real, FrictionCoeficient, Serialized)
  END_OPENFDM_OBJECT_DEF

SimpleContact::SimpleContact(const std::string& name) :
  Contact(name),
  mSpringConst(0),
  mDamperConstant(0),
  mFrictionCoef(0)
{
}

SimpleContact::~SimpleContact(void)
{
}

const real_type&
SimpleContact::getSpringConstant(void) const
{
  return mSpringConst;
}

void
SimpleContact::setSpringConstant(const real_type& springConst)
{
  mSpringConst = springConst;
}

const real_type&
SimpleContact::getDamperConstant(void) const
{
  return mDamperConstant;
}

void
SimpleContact::setDamperConstant(const real_type& damperConstant)
{
  mDamperConstant = damperConstant;
}

const real_type&
SimpleContact::getFrictionCoeficient(void) const
{
  return mFrictionCoef;
}

void
SimpleContact::setFrictionCoeficient(const real_type& frictionCoef)
{
  mFrictionCoef = frictionCoef;
}

// Compute the plane normal force.
real_type
SimpleContact::computeNormalForce(real_type compressLen,
                                  real_type compressVel) const
{
  return compressLen*mSpringConst - mDamperConstant*compressVel;
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
