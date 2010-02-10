/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "LinearSpringDamper.h"

#include "Model.h"
#include "PortValueList.h"
#include "TypeInfo.h"
#include "Variant.h"
#include "Vector.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(LinearSpringDamper, Model)
  DEF_OPENFDM_PROPERTY(Real, SpringReference, Serialized)
  DEF_OPENFDM_PROPERTY(Real, SpringConstant, Serialized)
  DEF_OPENFDM_PROPERTY(Real, DamperConstant, Serialized)
  END_OPENFDM_OBJECT_DEF

LinearSpringDamper::LinearSpringDamper(const std::string& name) :
  Model(name),
  mPositionPort(this, "position", true),
  mVelocityPort(this, "velocity", true),
  mForcePort(this, "force"),
  mSpringReference(0),
  mSpringConstant(0),
  mDamperConstant(0)
{
}

LinearSpringDamper::~LinearSpringDamper(void)
{
}

void
LinearSpringDamper::output(const Task&, const DiscreteStateValueVector&,
                           const ContinousStateValueVector&,
                           PortValueList& portValues) const
{
  real_type position = portValues[mPositionPort];
  real_type vel = portValues[mVelocityPort];
  real_type displacement = mSpringReference - position;
  portValues[mForcePort] = mSpringConstant*displacement - vel*mDamperConstant;
}

const real_type&
LinearSpringDamper::getSpringReference(void) const
{
  return mSpringReference;
}

void
LinearSpringDamper::setSpringReference(const real_type& springReference)
{
  mSpringReference = springReference;
}

const real_type&
LinearSpringDamper::getSpringConstant(void) const
{
  return mSpringConstant;
}

void
LinearSpringDamper::setSpringConstant(const real_type& springConstant)
{
  mSpringConstant = springConstant;
}

const real_type&
LinearSpringDamper::getDamperConstant(void) const
{
  return mDamperConstant;
}

void
LinearSpringDamper::setDamperConstant(const real_type& damperConstant)
{
  mDamperConstant = damperConstant;
}

} // namespace OpenFDM
