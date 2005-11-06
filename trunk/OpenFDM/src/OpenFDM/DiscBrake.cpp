/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Model.h"
#include "Vector.h"
#include "DiscBrake.h"

namespace OpenFDM {

DiscBrake::DiscBrake(const std::string& name) :
  LineForce(name),
  mFrictionConstant(-1)
{
  setNumInputPorts(1);
  setInputPortName(0, "brakePressure");
}

DiscBrake::~DiscBrake(void)
{
}

void
DiscBrake::output(const TaskInfo& taskInfo)
{
  real_type brakeInput = getInputPort(0).getValue().toReal();
  setForce(getVel()*(-1e1 + brakeInput*mFrictionConstant));
}

real_type
DiscBrake::getFrictionConstant(void) const
{
  return mFrictionConstant;
}

void
DiscBrake::setFrictionConstant(real_type frictionConstant)
{
  mFrictionConstant = frictionConstant;
}

} // namespace OpenFDM
