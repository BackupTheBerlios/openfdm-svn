/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Model.h"
#include "Vector.h"
#include "DiscBrake.h"

namespace OpenFDM {

DiscBrake::DiscBrake(const std::string& name) :
  Model(name),
  mMinForce(0),
  mMaxForce(1)
{
  setDirectFeedThrough(true);

  setNumContinousStates(1);

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

  // start with zero friction force
  mZ = 0;

  return true;
}

void
DiscBrake::output(const TaskInfo& taskInfo)
{
  real_type sigma = 100;
  real_type brakeInput = mBrakePressurePort.getRealValue();
  real_type vel = mVelocityPort.getRealValue();
  // with this sigma the model is already very crisp and reaches the
  // maximum force relatively fast, thus we do not need to make it even faster
  // with higher speeds
  vel = sign(vel)*min(1.0, fabs(vel));
  // the time derivative of the friction state
  mZDeriv = vel - sigma*fabs(vel)*mZ;
  // this is to limit the stiffness of this model
  mZDeriv = sign(mZDeriv)*min(10.0, fabs(mZDeriv));
  // now the output force, modulate with the brake input
  mForce = -interpolate(brakeInput, 0.0, mMinForce, 1.0, mMaxForce)*sigma*mZ;
}

void
DiscBrake::setState(const StateStream& state)
{
  state.readSubState(mZ);
}

void
DiscBrake::getState(StateStream& state) const
{
  state.writeSubState(mZ);
}

void
DiscBrake::getStateDeriv(StateStream& stateDeriv)
{
  stateDeriv.writeSubState(mZDeriv);
}

const real_type&
DiscBrake::getForce(void) const
{
  return mForce;
}

const real_type&
DiscBrake::getMinForce(void) const
{
  return mMinForce;
}

void
DiscBrake::setMinForce(const real_type& minForce)
{
  mMinForce = minForce;
}

const real_type&
DiscBrake::getMaxForce(void) const
{
  return mMaxForce;
}

void
DiscBrake::setMaxForce(const real_type& maxForce)
{
  mMaxForce = maxForce;
}

} // namespace OpenFDM
