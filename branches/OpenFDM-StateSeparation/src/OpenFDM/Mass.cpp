/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Mass.h"

#include "PortValueList.h"
#include "Transform.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Mass, Interact)
  DEF_OPENFDM_PROPERTY(Vector3, Position, Serialized)
  DEF_OPENFDM_PROPERTY(Inertia, Inertia, Serialized)
  DEF_OPENFDM_PROPERTY(Real, Mass, Serialized)
  END_OPENFDM_OBJECT_DEF

Mass::Mass(const std::string& name, const real_type& mass,
           const InertiaMatrix& inertia, const Vector3& position) :
  Interact(name),
  mMechanicLink(newMechanicLink("link")),
  mMass(mass),
  mInertia(inertia),
  mPosition(position)
{
}

Mass::~Mass(void)
{
}

void
Mass::initDesignPosition(PortValueList&) const
{
}

void
Mass::articulation(const Task&, const ContinousStateValueVector&,
                   PortValueList& portValues, Matrix&) const
{
  // FIXME: Hardcoding that gravity happens in the roots??
  Vector3 gravity = Vector3::zeros();
  Vector6 force = Vector6(Vector3::zeros(), gravity);

  Vector3 position = mPosition - portValues[mMechanicLink].getDesignPosition();
  SpatialInertia I = inertiaFrom(position, SpatialInertia(mInertia, mMass));

  // FIXME: do we really need that in the mass
  // I did search for a while until I found that missing term here ...
  Vector6 iv = portValues[mMechanicLink].getFrame().getSpVel();
  Vector6 Jiv = I*iv;
  force += Vector6(cross(iv.getAngular(), Jiv.getAngular()) +
                   cross(iv.getLinear(), Jiv.getLinear()),
                   cross(iv.getAngular(), Jiv.getLinear()));


  portValues[mMechanicLink].applyInertia(I);
  portValues[mMechanicLink].applyForce(forceFrom(position, force));
}

const InertiaMatrix&
Mass::getInertia(void) const
{
  return mInertia;
}

void
Mass::setInertia(const InertiaMatrix& inertia)
{
  mInertia = inertia;
}

const real_type&
Mass::getMass() const
{
  return mMass;
}

void
Mass::setMass(const real_type& mass)
{
  mMass = mass;
}

const Vector3&
Mass::getPosition(void) const
{
  return mPosition;
}

void
Mass::setPosition(const Vector3& position)
{
  mPosition = position;
}

} // namespace OpenFDM
