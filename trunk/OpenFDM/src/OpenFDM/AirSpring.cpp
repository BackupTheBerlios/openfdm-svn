/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "AirSpring.h"

#include <string>

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(AirSpring, Model)
  DEF_OPENFDM_PROPERTY(Real, Area, Serialized)
  DEF_OPENFDM_PROPERTY(Real, PushPressure, Serialized)
  DEF_OPENFDM_PROPERTY(Real, PullPressure, Serialized)
  DEF_OPENFDM_PROPERTY(Real, MaxCompression, Serialized)
  DEF_OPENFDM_PROPERTY(Real, MinCompression, Serialized)
  DEF_OPENFDM_PROPERTY(Real, MaxDamperConstant, Serialized)
  DEF_OPENFDM_PROPERTY(Real, MinDamperConstant, Serialized)
  END_OPENFDM_OBJECT_DEF

AirSpring::AirSpring(const std::string& name) :
  Model(name),
  mPushPressure(2e5),
  mPullPressure(1e5),
  mArea(0),
  mMinCompression(0),
  mMaxCompression(0),
  mMinDamperConstant(0),
  mMaxDamperConstant(0),
  mGamma(1.4)
{
  setDirectFeedThrough(true);

  setNumInputPorts(2);
  setInputPortName(0, "position");
  setInputPortName(1, "velocity");
  
  setNumOutputPorts(1);
  setOutputPort(0, "force", this, &AirSpring::getForce);
}

AirSpring::~AirSpring(void)
{
}

bool
AirSpring::init(void)
{
  mPositionPort = getInputPort(0)->toRealPortHandle();
  if (!mPositionPort.isConnected()) {
    Log(Model, Error) << "Initialization of AirSpring model \"" << getName()
                      << "\" failed: Input port \"" << getInputPortName(0)
                      << "\" is not connected!" << endl;
    return false;
  }

  mVelocityPort = getInputPort(1)->toRealPortHandle();
  if (!mVelocityPort.isConnected()) {
    Log(Model, Error) << "Initialization of AirSpring model \"" << getName()
                      << "\" failed: Input port \"" << getInputPortName(1)
                      << "\" is not connected!" << endl;
    return false;
  }

  return Model::init();
}

void
AirSpring::output(const TaskInfo& taskInfo)
{
  real_type position = mPositionPort.getRealValue();
  real_type vel = mVelocityPort.getRealValue();

  real_type maxDisp = mMaxCompression - mMinCompression;
  real_type pullDisp = mMaxCompression - position;
  real_type pushDisp = position - mMinCompression;
  
  real_type pullDispRatio = pullDisp/maxDisp;
  real_type pushDispRatio = pushDisp/maxDisp;
  
  
  pullDispRatio = max(min(pullDispRatio, 0.95), 0.0);
  pushDispRatio = max(min(pushDispRatio, 0.95), 0.0);
  
  real_type pullPressure = mPullPressure/(1-pow(pullDispRatio, mGamma));
  real_type pushPressure = mPushPressure/(1-pow(pushDispRatio, mGamma));
  
  // The output force is the pressure difference times the piston area
  mForce = sign(maxDisp)*mArea*(pullPressure - pushPressure);
  // Add a position dependent damping force
  // That sign of the area is just a handy hack to determine
  // the polarity of the output value
  mForce += sign(mArea)*vel*interpolate(position,
                                        mMinCompression, mMinDamperConstant,
                                        mMaxCompression, mMaxDamperConstant);
}

const real_type&
AirSpring::getForce(void) const
{
  return mForce;
}

const real_type&
AirSpring::getPushPressure(void) const
{
  return mPushPressure;
}

void
AirSpring::setPushPressure(const real_type& pushPressure)
{
  mPushPressure = pushPressure;
}

const real_type&
AirSpring::getPullPressure(void) const
{
  return mPullPressure;
}

void
AirSpring::setPullPressure(const real_type& pullPressure)
{
  mPullPressure = pullPressure;
}

const real_type&
AirSpring::getArea(void) const
{
  return mArea;
}

void
AirSpring::setArea(const real_type& area)
{
  mArea = area;
}

const real_type&
AirSpring::getMaxCompression(void) const
{
  return mMaxCompression;
}

void
AirSpring::setMaxCompression(const real_type& maxCompression)
{
  mMaxCompression = maxCompression;
}

const real_type&
AirSpring::getMinCompression(void) const
{
  return mMinCompression;
}

void
AirSpring::setMinCompression(const real_type& minCompression)
{
  mMinCompression = minCompression;
}

const real_type&
AirSpring::getMaxDamperConstant(void) const
{
  return mMaxDamperConstant;
}

void
AirSpring::setMaxDamperConstant(const real_type& maxDamperConstant)
{
  mMaxDamperConstant = maxDamperConstant;
}

const real_type&
AirSpring::getMinDamperConstant(void) const
{
  return mMinDamperConstant;
}

void
AirSpring::setMinDamperConstant(const real_type& minDamperConstant)
{
  mMinDamperConstant = minDamperConstant;
}

real_type
AirSpring::getGamma(void) const
{
  return mGamma;
}

void
AirSpring::setGamma(real_type gamma)
{
  mGamma = gamma;
}

} // namespace OpenFDM
