/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Mass.h"

namespace OpenFDM {

Mass::Mass(const std::string& name, const SpatialInertia& inertia) :
  Interact(name, 1),
  mInertia(inertia),
  mUntransformedInertia(inertia),
  mPosition(Vector3::zeros())
{
  addProperty("posoition", Property(this, &Mass::getPosition, &Mass::setPosition));
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
