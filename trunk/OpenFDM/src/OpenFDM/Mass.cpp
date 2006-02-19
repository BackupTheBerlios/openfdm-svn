/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Mass.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Mass, Interact)
  DEF_OPENFDM_PROPERTY(Vector3, Position, Serialized)
  END_OPENFDM_OBJECT_DEF

Mass::Mass(const std::string& name, const SpatialInertia& inertia) :
  Interact(name, 1),
  mInertia(inertia),
  mUntransformedInertia(inertia),
  mPosition(Vector3::zeros())
{
}

Mass::~Mass(void)
{
}

void
Mass::interactWith(RigidBody* rigidBody)
{
  rigidBody->contributeLocalInertia(mInertia);
}

void
Mass::setInertia(real_type mass)
{
  setInertia(SpatialInertia(mass));
}

void
Mass::setInertia(real_type mass, const InertiaMatrix& inertia)
{
  setInertia(SpatialInertia(inertia, mass));
}

void
Mass::setInertia(const SpatialInertia& I)
{
  mUntransformedInertia = I;
  mInertia = inertiaFrom(mPosition, mUntransformedInertia);
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
  mInertia = inertiaFrom(mPosition, mUntransformedInertia);
}

} // namespace OpenFDM
