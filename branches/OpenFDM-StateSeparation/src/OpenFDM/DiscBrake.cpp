/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "DiscBrake.h"

#include "Model.h"
#include "TypeInfo.h"
#include "Variant.h"
#include "Vector.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(DiscBrake, Model)
  DEF_OPENFDM_PROPERTY(Real, MinForce, Serialized)
  DEF_OPENFDM_PROPERTY(Real, MaxForce, Serialized)
  END_OPENFDM_OBJECT_DEF

DiscBrake::DiscBrake(const std::string& name) :
  Model(name),
  mMinForce(0),
  mMaxForce(1),
  mSigma(100),
  mZStateInfo(new Vector1StateInfo),
  mBrakeInputPort(this, "brakeInput", true),
  mVelocityPort(this, "velocity", true),
  mForcePort(this, "force")
{
  addContinousStateInfo(mZStateInfo);
}

DiscBrake::~DiscBrake(void)
{
}

void
DiscBrake::init(const Task&, DiscreteStateValueVector&,
                ContinousStateValueVector& state, const PortValueList&) const
{
  state[*mZStateInfo](0, 0) = 0;
}

void
DiscBrake::output(const Task&, const DiscreteStateValueVector&,
                  const ContinousStateValueVector& state,
                  PortValueList& portValues) const
{
  real_type brakeInput = portValues[mBrakeInputPort];
  real_type z = state[*mZStateInfo](0, 0);
  // now the output force, modulate with the brake input
  portValues[mForcePort] = -interpolate(brakeInput, real_type(0), mMinForce,
                                        real_type(1), mMaxForce)*mSigma*z;
}

void
DiscBrake::derivative(const DiscreteStateValueVector&,
                      const ContinousStateValueVector& state,
                      const PortValueList& portValues,
                      ContinousStateValueVector& deriv) const
{
  real_type z = state[*mZStateInfo](0, 0);
  real_type vel = portValues[mVelocityPort];
  // with this mSigma the model is already very crisp and reaches the
  // maximum force relatively fast, thus we do not need to make it even faster
  // with higher speeds
//   vel = saturate(vel, real_type(1));
  vel = smoothSaturate(vel, real_type(1));
  // the time derivative of the friction state
  real_type zDeriv = vel - mSigma*fabs(vel)*z;
  // this is to limit the stiffness of this model
//   zDeriv = saturate(zDeriv, real_type(10));
  zDeriv = smoothSaturate(zDeriv, real_type(10));
  deriv[*mZStateInfo](0, 0) = zDeriv;
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
