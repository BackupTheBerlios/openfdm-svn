/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Mass.h"

#include "PortValueList.h"
#include "Transform.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Mass, SingleLinkInteract)
  DEF_OPENFDM_PROPERTY(Vector3, Position, Serialized)
  DEF_OPENFDM_PROPERTY(Inertia, Inertia, Serialized)
  DEF_OPENFDM_PROPERTY(Real, Mass, Serialized)
  END_OPENFDM_OBJECT_DEF

Mass::Mass(const std::string& name, const real_type& mass,
           const InertiaMatrix& inertia, const Vector3& position) :
  SingleLinkInteract(name),
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
Mass::articulation(const Task&, const Environment& environment,
                   const ContinousStateValueVector&,
                   PortValueList& portValues) const
{
  // The position of the mass point wrt its parent link frame
  // FIXME precompute that
  Vector3 position = mPosition - portValues[mMechanicLink].getDesignPosition();

  // The gravity force that applies to this mass
  Vector3 refPosition = portValues[mMechanicLink].getFrame().posToRef(position);
  Vector3 gravity = environment.getGravityAcceleration(refPosition);
  gravity = mMass*portValues[mMechanicLink].getFrame().rotFromRef(gravity);
  // The gravity force at the coordinate system of the parent link
  Vector6 force = forceFrom(position, gravity);

  // The inertia at the coordinate system of the parent link
  // FIXME precompute that
  SpatialInertia I = inertiaFrom(position, SpatialInertia(mInertia, mMass));

  // FIXME: do we really need that in the mass
  // I did search for a while until I found that missing term here ...
  Vector6 v = portValues[mMechanicLink].getFrame().getSpVel();
  Vector6 Iv = I*v;
  Vector6 vIv = Vector6(cross(v.getAngular(), Iv.getAngular()) +
                        cross(v.getLinear(), Iv.getLinear()),
                        cross(v.getAngular(), Iv.getLinear()));

  portValues[mMechanicLink].addInertia(I);
  portValues[mMechanicLink].addForce(Vector6(vIv - force));
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
