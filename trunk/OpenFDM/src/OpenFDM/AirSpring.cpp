/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include <string>

#include "AirSpring.h"

namespace OpenFDM {

AirSpring::AirSpring(const std::string& name) :
  Model(name),
  mPushPressure(2e5),
  mPullPressure(1e5),
  mArea(0),
  mMinCompression(0),
  mMaxCompression(0),
  mMinDamperConstant(0),
  mMaxDamperConstant(0),
  mGamma(1.3)
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

  return true;
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

real_type
AirSpring::getPushPressure(void) const
{
  return mPushPressure;
}

void
AirSpring::setPushPressure(real_type pushPressure)
{
  mPushPressure = pushPressure;
}

real_type
AirSpring::getPullPressure(void) const
{
  return mPullPressure;
}

void
AirSpring::setPullPressure(real_type pullPressure)
{
  mPullPressure = pullPressure;
}

real_type
AirSpring::getArea(void) const
{
  return mArea;
}

void
AirSpring::setArea(real_type area)
{
  mArea = area;
}

real_type
AirSpring::getMaxCompression(void) const
{
  return mMaxCompression;
}

void
AirSpring::setMaxCompression(real_type maxCompression)
{
  mMaxCompression = maxCompression;
}

real_type
AirSpring::getMinCompression(void) const
{
  return mMinCompression;
}

void
AirSpring::setMinCompression(real_type minCompression)
{
  mMinCompression = minCompression;
}

real_type
AirSpring::getMaxDamperConstant(void) const
{
  return mMaxDamperConstant;
}

void
AirSpring::setMaxDamperConstant(real_type maxDamperConstant)
{
  mMaxDamperConstant = maxDamperConstant;
}

real_type
AirSpring::getMinDamperConstant(void) const
{
  return mMinDamperConstant;
}

void
AirSpring::setMinDamperConstant(real_type minDamperConstant)
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
