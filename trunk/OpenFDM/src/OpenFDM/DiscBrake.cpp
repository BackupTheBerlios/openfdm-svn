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
  Port* port = getInputPort(0);
  real_type brakeInput = 0;
  if (port->isConnected()) {
    RealPortHandle rh = port->toRealPortHandle();
    brakeInput = rh.getRealValue();
  }
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
