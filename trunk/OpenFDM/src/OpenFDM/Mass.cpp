/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Frame.h"
#include "Mass.h"
#include "Visitor.h"
#include "ConstVisitor.h"

namespace OpenFDM {

Mass::Mass(const SpatialInertia& inertia, const std::string& name)
  : MultiBodyModel(name), _inertia(inertia)
{
}

Mass::~Mass(void)
{
}

void
Mass::accept(Visitor& visitor)
{
  visitor.apply(*this);
}

void
Mass::accept(ConstVisitor& visitor) const
{
  visitor.apply(*this);
}

Mass*
Mass::toMass(void)
{
  return this;
}

const Mass*
Mass::toMass(void) const
{
  return this;
}

void
Mass::setInertia(real_type mass)
{
  _inertia = SpatialInertia(mass);
}

void
Mass::setInertia(real_type mass, const InertiaMatrix& inertia)
{
  _inertia = SpatialInertia(inertia, mass);
}

void
Mass::setInertia(const SpatialInertia& I)
{
  _inertia = I;
}

void
Mass::contributeInertia(SpatialInertia& I) const
{
  I += _inertia;
}

} // namespace OpenFDM
