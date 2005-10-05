/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Model.h"
#include "MultiBodyModel.h"
#include "Visitor.h"
#include "ConstVisitor.h"

namespace OpenFDM {

MultiBodyModel::MultiBodyModel(const std::string& name) :
  Model(name)
{
}

MultiBodyModel::~MultiBodyModel(void)
{
}

void
MultiBodyModel::accept(Visitor& visitor)
{
  visitor.apply(*this);
}

void
MultiBodyModel::traverse(Visitor& visitor)
{
}

void
MultiBodyModel::accept(ConstVisitor& visitor) const
{
  visitor.apply(*this);
}

void
MultiBodyModel::traverse(ConstVisitor& visitor) const
{
}

Mass*
MultiBodyModel::toMass(void)
{
  return 0;
}

const Mass*
MultiBodyModel::toMass(void) const
{
  return 0;
}

Force*
MultiBodyModel::toForce(void)
{
  return 0;
}

const Force*
MultiBodyModel::toForce(void) const
{
  return 0;
}

Joint*
MultiBodyModel::toJoint(void)
{
  return 0;
}

const Joint*
MultiBodyModel::toJoint(void) const
{
  return 0;
}

} // namespace OpenFDM
