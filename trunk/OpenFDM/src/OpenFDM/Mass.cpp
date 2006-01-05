/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Mass.h"

namespace OpenFDM {

Mass::Mass(const std::string& name, const SpatialInertia& inertia) :
  Interact(name, 1),
  mInertia(inertia)
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
  mInertia = SpatialInertia(mass);
}

void
Mass::setInertia(real_type mass, const InertiaMatrix& inertia)
{
  mInertia = SpatialInertia(inertia, mass);
}

void
Mass::setInertia(const SpatialInertia& I)
{
  mInertia = I;
}

} // namespace OpenFDM
