/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "ConstSystem.h"

namespace OpenFDM {

ConstSystem::ConstSystem(const std::string& name, const Matrix& value) :
  Model(name), mValue(value)
{
  addProperty("value", Property(this, &ConstSystem::getValue, &ConstSystem::setValue));

  setNumOutputPorts(1);
  setOutputPort(0, "output", Property(this, &ConstSystem::getValue));
}

ConstSystem::~ConstSystem(void)
{
}

const Matrix&
ConstSystem::getValue(void) const
{
  return mValue;
}

void
ConstSystem::setValue(const Matrix& value)
{
  mValue = value;
}

} // namespace OpenFDM
