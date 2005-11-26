/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Model.h"
#include "Vector.h"
#include "LineActuator.h"

namespace OpenFDM {

LineActuator::LineActuator(const std::string& name) :
  LineForce(name),
  mProportionalGain(1),
  mDerivativeGain(0)
{
  setNumInputPorts(1);
  setInputPortName(0, "targetPosition");
}

LineActuator::~LineActuator(void)
{
}

void
LineActuator::output(const TaskInfo& taskInfo)
{
  real_type posInput = getInputPort(0)->getValue().toReal();
  real_type displacement = getPosition() - posInput;
  setForce(mProportionalGain*displacement + getVel()*mDerivativeGain);
}

real_type
LineActuator::getProportionalGain(void) const
{
  return mProportionalGain;
}

void
LineActuator::setProportionalGain(real_type proportionalGain)
{
  mProportionalGain = proportionalGain;
}

real_type
LineActuator::getDerivativeGain(void) const
{
  return mDerivativeGain;
}

void
LineActuator::setDerivativeGain(real_type derivativeGain)
{
  mDerivativeGain = derivativeGain;
}

} // namespace OpenFDM
