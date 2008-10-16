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
  mPosition(position),
  mSpatialInertia(0)
{
  setInertia(mMass, mInertia, mPosition);
}

Mass::~Mass(void)
{
}

void
Mass::articulation(const Task&, const ContinousStateValueVector&,
                   PortValueList& portValues, MechanicContext&) const
{
  // Hardcoding that gravity happens in the roots??
  // Vectro3 position = portValues[mMechanicLink].mPosition;
  Vector3 gravity = Vector3::zeros();

  portValues[mMechanicLink].applyInertia(mSpatialInertia);
  portValues[mMechanicLink].applyForce(gravity);
}

const InertiaMatrix&
Mass::getInertia(void) const
{
  return mInertia;
}

void
Mass::setInertia(const InertiaMatrix& inertia)
{
  setInertia(mMass, inertia, mPosition);
}

const real_type&
Mass::getMass() const
{
  return mMass;
}

void
Mass::setMass(const real_type& mass)
{
  setInertia(mass, mInertia, mPosition);
}

const Vector3&
Mass::getPosition(void) const
{
  return mPosition;
}

void
Mass::setPosition(const Vector3& position)
{
  setInertia(mMass, mInertia, position);
}

void
Mass::setInertia(const real_type& mass, const InertiaMatrix& inertia,
                 const Vector3& position)
{
  mMass = mass;
  mInertia = inertia;
  mPosition = position;
  mSpatialInertia = inertiaFrom(mPosition, SpatialInertia(mInertia, mMass));
}

} // namespace OpenFDM
