/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "Actuator.h"

#include "Types.h"
#include "Object.h"
#include "Model.h"

namespace OpenFDM {

Actuator::Actuator(const std::string& name) :
  Model(name),
  mInputPort(newRealInputPort("input", false)),
  mPositionPort(newRealOutputPort("position")),
  mVelocityPort(newRealOutputPort("velocity")),
  mPositionStateInfo(new Vector1StateInfo),
  mVelocityStateInfo(new Vector1StateInfo),
  mInitialPosition(0),
  mInitialVelocity(0),
  mMinDeflection(-Limits<real_type>::max()),
  mMaxDeflection(Limits<real_type>::max()),
  mMaxRate(Limits<real_type>::max()),
  mEigenFreq(15),
  mDampingRatio(0.7)
{
  addContinousStateInfo(mPositionStateInfo);
  addContinousStateInfo(mVelocityStateInfo);
}

Actuator::~Actuator()
{
}

void
Actuator::init(const Task&, DiscreteStateValueVector& discreteState,
               ContinousStateValueVector& continousState,
               const PortValueList& portValueList) const
{
  continousState[*mPositionStateInfo] = mInitialPosition;
  continousState[*mVelocityStateInfo] = mInitialVelocity;
}

void
Actuator::output(const Task&,const DiscreteStateValueVector&,
                 const ContinousStateValueVector& states,
                 PortValueList& portValues) const
{
  portValues[mPositionPort] = states[*mPositionStateInfo](0);
  portValues[mVelocityPort] = states[*mVelocityStateInfo](0);
}

void
Actuator::derivative(const DiscreteStateValueVector&,
                     const ContinousStateValueVector& states,
                     const PortValueList& portValues,
                     ContinousStateValueVector& deriv) const
{
  // The desired position
  real_type desiredPos = portValues[mInputPort];
  // Apply the actuator limits.
  desiredPos = min(desiredPos, mMaxDeflection);
  desiredPos = max(desiredPos, mMinDeflection);

  // Compute the position error
  real_type positionError = desiredPos - states[*mPositionStateInfo](0);
    
  // The velocity at eigenfrequency
  real_type omega = 1/mEigenFreq;
  
  // Apply the rate limits
  positionError = min(positionError, 2*mDampingRatio*mMaxRate/omega);
  positionError = max(positionError, -2*mDampingRatio*mMaxRate/omega);

  // Compute acceleration
  real_type accel = positionError*omega*omega - 2*omega*mDampingRatio;

  // And return the state derivative
  deriv[*mPositionStateInfo] = states[*mVelocityStateInfo];
  deriv[*mVelocityStateInfo] = accel;
}

} // namespace OpenFDM
