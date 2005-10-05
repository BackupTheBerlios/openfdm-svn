/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Force.h"

namespace OpenFDM {

Force::Force(const std::string& name)
  : MultiBodyModel(name)
{
}

Force::~Force(void)
{
}

void
Force::accept(Visitor& visitor)
{
  visitor.apply(*this);
}

void
Force::accept(ConstVisitor& visitor) const
{
  visitor.apply(*this);
}

Force*
Force::toForce(void)
{
  return this;
}

const Force*
Force::toForce(void) const
{
  return this;
}

} // namespace OpenFDM
