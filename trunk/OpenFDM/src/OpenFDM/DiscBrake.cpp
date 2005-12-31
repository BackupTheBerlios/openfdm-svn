/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Model.h"
#include "Vector.h"
#include "DiscBrake.h"

namespace OpenFDM {

DiscBrake::DiscBrake(const std::string& name) :
  Model(name),
  mFrictionConstant(-1)
{
  setDirectFeedThrough(true);

  setNumInputPorts(2);
  setInputPortName(0, "brakePressure");
  setInputPortName(1, "velocity");
  
  setNumOutputPorts(1);
  setOutputPort(0, "force", this, &DiscBrake::getForce);
}

DiscBrake::~DiscBrake(void)
{
}

bool
DiscBrake::init(void)
{
  if (!getInputPort(0)->isConnected()) {
    Log(Model, Error) << "Initialization of DiscBrake model \"" << getName()
                      << "\" failed: Input port \"" << getInputPortName(0)
                      << "\" is not connected!" << endl;
    return false;
  }
  mBrakePressurePort = getInputPort(0)->toRealPortHandle();

  if (!getInputPort(1)->isConnected()) {
    Log(Model, Error) << "Initialization of DiscBrake model \"" << getName()
                      << "\" failed: Input port \"" << getInputPortName(1)
                      << "\" is not connected!" << endl;
    return false;
  }
  mVelocityPort = getInputPort(1)->toRealPortHandle();

  return true;
}

void
DiscBrake::output(const TaskInfo& taskInfo)
{
  real_type brakeInput = mBrakePressurePort.getRealValue();
  real_type vel = mVelocityPort.getRealValue();
  /// Hmm, this seems to be an intermediate model for a disc brake ...
  mForce = vel*(-1e1 + brakeInput*mFrictionConstant);
}

const real_type&
DiscBrake::getForce(void) const
{
  return mForce;
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
