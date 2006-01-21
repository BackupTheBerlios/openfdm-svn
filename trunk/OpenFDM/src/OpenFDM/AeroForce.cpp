/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Frame.h"
#include "Force.h"
#include "Atmosphere.h"
#include "AeroForce.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(AeroForce)
  END_OPENFDM_OBJECT_DEF

AeroForce::AeroForce(const std::string& name)
  : ExternalForce(name)
{
  mWingSpan = 0.0;
  mWingArea = 0.0;
  mCoord = 0.0;

  dirtyAll();

  addStoredProperty("wingSpan",
                    Property(this, &AeroForce::getWingSpan, &AeroForce::setWingSpan));
  addStoredProperty("wingArea",
                    Property(this, &AeroForce::getWingArea, &AeroForce::setWingArea));
  addStoredProperty("coord",
                    Property(this, &AeroForce::getCoord, &AeroForce::setCoord));
  addOutputPort("wingSpan", this, &AeroForce::getWingSpan);
  addOutputPort("wingArea", this, &AeroForce::getWingArea);
  addOutputPort("coord", this, &AeroForce::getCoord);

  addOutputPort("altitude", this, &AeroForce::getAltitude);
  addOutputPort("aboveGroundLevel", this, &AeroForce::getAboveGroundLevel);

  addOutputPort("trueSpeed", this, &AeroForce::getTrueSpeed);
  addOutputPort("dynamicPressure", this, &AeroForce::getDynamicPressure);
  addOutputPort("alpha", this, &AeroForce::getAlpha);
  addOutputPort("alphaDot", this, &AeroForce::getAlphaDot);
  addOutputPort("beta", this, &AeroForce::getBeta);
  addOutputPort("betaDot", this, &AeroForce::getBetaDot);
//   addOutputPort("mach", this, &AeroForce::getMach);
  addOutputPort("machNumber", this, &AeroForce::getMachNumber);
  addOutputPort("trueSpeedUW", this, &AeroForce::getTrueSpeedUW);
  addOutputPort("wingSpanOver2Speed", this, &AeroForce::getWingSpanOver2Speed);
  addOutputPort("coordOver2Speed", this, &AeroForce::getCoordOver2Speed);
  addOutputPort("hOverWingSpan", this, &AeroForce::getHOverWingSpan);

  addOutputPort("pressure", this, &AeroForce::getPressure);
  addOutputPort("density", this, &AeroForce::getDensity);
  addOutputPort("soundSpeed", this, &AeroForce::getSoundSpeed);
  addOutputPort("temperature", this, &AeroForce::getTemperature);

  addOutputPort("u", this, &AeroForce::getBodyU);
  addOutputPort("v", this, &AeroForce::getBodyV);
  addOutputPort("w", this, &AeroForce::getBodyW);
  addOutputPort("p", this, &AeroForce::getBodyP);
  addOutputPort("q", this, &AeroForce::getBodyQ);
  addOutputPort("r", this, &AeroForce::getBodyR);

  setNumInputPorts(6);
  setInputPortName(0, "roll");
  setInputPortName(1, "pitch");
  setInputPortName(2, "yaw");
  setInputPortName(3, "drag");
  setInputPortName(4, "side");
  setInputPortName(5, "lift");
}

AeroForce::~AeroForce(void)
{
}

bool
AeroForce::init(void)
{
  mEnvironment = getEnvironment();
  if (!mEnvironment)
    return false;

  if (getInputPort("roll")->isConnected())
    mBodyAxisTorque[0] = getInputPort("roll")->toRealPortHandle();
  else
    mBodyAxisTorque[0] = 0;
  if (getInputPort("pitch")->isConnected())
    mBodyAxisTorque[1] = getInputPort("pitch")->toRealPortHandle();
  else
    mBodyAxisTorque[1] = 0;
  if (getInputPort("yaw")->isConnected())
    mBodyAxisTorque[2] = getInputPort("yaw")->toRealPortHandle();
  else
    mBodyAxisTorque[2] = 0;

  if (getInputPort("drag")->isConnected())
    mStabilityAxisForce[0] = getInputPort("drag")->toRealPortHandle();
  else
    mStabilityAxisForce[0] = 0;
  if (getInputPort("side")->isConnected())
    mStabilityAxisForce[1] = getInputPort("side")->toRealPortHandle();
  else
    mStabilityAxisForce[1] = 0;
  if (getInputPort("lift")->isConnected())
    mStabilityAxisForce[2] = getInputPort("lift")->toRealPortHandle();
  else
    mStabilityAxisForce[2] = 0;

  return ExternalForce::init();
}

void
AeroForce::output(const TaskInfo& taskInfo)
{
  if (nonZeroIntersection(taskInfo.getSampleTimeSet(),
                          SampleTime::PerTimestep)) {
    Log(Model, Debug) << "AeroForce::output(): \"" << getName()
                      << "\" computing ground plane below" << endl;
    real_type t = taskInfo.getTime();
    mGroundVal = mEnvironment->getGround()->getGroundPlane(t, getRefPosition());
  }
  dirtyAll();

  // FIXME: they can be computed cheaper ...
  real_type ca = cos(getAlpha());
  real_type sa = sin(getAlpha());
  real_type cb = cos(getBeta());
  real_type sb = sin(getBeta());
  Matrix33 Ts2b(-ca*cb, -ca*sb,  sa,
                   -sb,     cb,   0,
                -sa*cb, -sa*sb, -ca);

  // This is simple here. Just collect all summands ...
  Vector3 stabilityForce = Vector3::zeros();
  /// Lift points upward
  /// Drag points backward
  for (int i = 0; i < 3; ++i)
    if (mStabilityAxisForce[i].isConnected())
      stabilityForce(i+1) = mStabilityAxisForce[i].getRealValue();

  Vector3 bodyTorque = Vector3::zeros();
  for (int i = 0; i < 3; ++i)
    if (mBodyAxisTorque[i].isConnected())
      bodyTorque(i+1) = mBodyAxisTorque[i].getRealValue();

  Vector6 force(bodyTorque, Ts2b*stabilityForce);
  Log(ArtBody, Debug3) << "AeroForce::output() "
                       << trans(force) << endl;
  setForce(force);
}

void
AeroForce::setWingSpan(const real_type& winSpan)
{
  mWingSpan = winSpan;
}

const real_type&
AeroForce::getWingSpan(void) const
{
  return mWingSpan;
}

void
AeroForce::setWingArea(const real_type& winArea)
{
  mWingArea = winArea;
}

const real_type&
AeroForce::getWingArea(void) const
{
  return mWingArea;
}

void
AeroForce::setCoord(const real_type& coord)
{
  mCoord = coord;
}

const real_type&
AeroForce::getCoord(void) const
{
  return mCoord;
}

const Vector3&
AeroForce::getRefPosition(void) const
{
  if (mDirtyRefPosition) {
    // Get the position in the earth centered coordinate frame.
    mRefPosition = mMountFrame->getRefPosition();
    mDirtyRefPosition = false;
  }
  Log(ArtBody, Debug3) << "AeroForce::getRefPosition()"
                       << trans(mRefPosition) << endl;
  return mRefPosition;
}

const Vector6&
AeroForce::getAirSpeed(void) const
{
  if (mDirtyAirSpeed) {
    // FIXME temporary workaround
    if (!mEnvironment) {
      const_cast<AeroForce*>(this)->mEnvironment = getEnvironment();
    }
    // Get the position in the earth centered coordinate frame.
    Vector3 windVel = mEnvironment->getWind()->getWindVel(getRefPosition());
    windVel = mMountFrame->rotFromRef(windVel);
    mAirSpeed = Vector6(Vector3::zeros(), windVel) + mMountFrame->getRefVel();
    mDirtyAirSpeed = false;
  }
  Log(ArtBody, Debug3) << "AeroForce::getAirSpeed()"
                       << trans(mAirSpeed) << endl;
  return mAirSpeed;
}

const Vector3&
AeroForce::getMach(void) const
{
  if (mDirtyMach) {
    mMach = (1/getSoundSpeed())*getAirSpeed().getLinear();
    mDirtyMach = false;
  }
  Log(ArtBody, Debug3) << "AeroForce::getMach()"
                       << trans(mMach) << endl;
  return mMach;
}

const real_type&
AeroForce::getTrueSpeed(void) const
{
  if (mDirtyTrueSpeed) {
    mTrueSpeed = norm(getAirSpeed().getLinear());
    mDirtyTrueSpeed = false;
  }
  Log(ArtBody, Debug3) << "AeroForce::getTrueSpeed()"
                       << mTrueSpeed << endl;
  return mTrueSpeed;
}

const real_type&
AeroForce::getEquivalentAirSpeed(void) const
{
  if (mDirtyEquivalentAirSpeed)
    computeCalEquAirspeed();
  Log(ArtBody, Debug3) << "AeroForce::getEquivalentAirSpeed()"
                       << mEquivalentAirSpeed << endl;
  return mEquivalentAirSpeed;
}

const real_type&
AeroForce::getCalibratedAirSpeed(void) const
{
  if (mDirtyCalibratedAirSpeed)
    computeCalEquAirspeed();
  Log(ArtBody, Debug3) << "AeroForce::getCalibratedAirSpeed()"
                       << mCalibratedAirSpeed << endl;
  return mCalibratedAirSpeed;
}

const real_type&
AeroForce::getDynamicPressure(void) const
{
  if (mDirtyDynamicPressure) {
    real_type V = getTrueSpeed();
    real_type rho = getDensity();
    mDynamicPressure = 0.5*rho*V*V;
  }
  Log(ArtBody, Debug3) << "AeroForce::getDynamicPressure()"
                       << mDynamicPressure << endl;
  return mDynamicPressure;
}

const real_type&
AeroForce::getAlpha(void) const
{
  if (mDirtyAlpha) {
    Vector3 V = getAirSpeed().getLinear();
    if (fabs(V(iU)) < Limits<real_type>::min())
      mAlpha = 0;
    else
      mAlpha = atan2(V(iW), V(iU));
    mDirtyAlpha = false;
  }
  Log(ArtBody, Debug3) << "AeroForce::getAlpha()" << mAlpha << endl;
  return mAlpha;
}

const real_type&
AeroForce::getAlphaDot(void) const
{
  if (mDirtyAlphaDot) {
  // The timederivative of alpha.
  // Here it is getting difficult. The acceleration is not well defined
  // at this time. But we just take the past one ...
//   real_type alphadot = 0;
//   if (airSpeedUW2 != 0)
//     alphadot = (airSpeed(1)*stabAccel(3)-airSpeed(3)*stabAccel(1))/airSpeedUW2;

    mAlphaDot = 0;
    mDirtyAlphaDot = false;
  }
  Log(ArtBody, Debug3) << "AeroForce::getAlphaDot() " << mAlphaDot << endl;
  return mAlphaDot;
}

const real_type&
AeroForce::getBeta(void) const
{
  if (mDirtyBeta) {
    real_type Vuw = getTrueSpeedUW();
    Vector3 V = getAirSpeed().getLinear();
    if (fabs(Vuw) < Limits<real_type>::min())
      mBeta = 0;
    else
      mBeta = atan2(V(iV), Vuw);
    
    mDirtyBeta = false;
  }
  Log(ArtBody, Debug3) << "AeroForce::getBeta() " << mBeta << endl;
  return mBeta;
}

const real_type&
AeroForce::getBetaDot(void) const
{
  if (mDirtyBetaDot) {
    mBetaDot = 0;
//   // The timederivative of beta.
//   // The same implicit dependency like with alphadot.
//   real_type betadot = 0;
//   if (airSpeedUW != 0 && trueSpeed != 0) {
//     real_type trueSpeedDot = dot(airSpeed, stabAccel)/trueSpeed;
//     betadot = (trueSpeed*stabAccel(2) - airSpeed(2)*trueSpeedDot);
//     betadot /= trueSpeed*airSpeedUW;
//   }

    mDirtyBetaDot = false;
  }
  Log(ArtBody, Debug3) << "AeroForce::getBetaDot() " << mBetaDot << endl;
  return mBetaDot;
}

const real_type&
AeroForce::getBodyU(void) const
{
  const Vector6& speed = getAirSpeed();
  return speed(4);
}

const real_type&
AeroForce::getBodyV(void) const
{
  const Vector6& speed = getAirSpeed();
  return speed(5);
}

const real_type&
AeroForce::getBodyW(void) const
{
  const Vector6& speed = getAirSpeed();
  return speed(6);
}

const real_type&
AeroForce::getBodyP(void) const
{
  const Vector6& speed = getAirSpeed();
  return speed(1);
}

const real_type&
AeroForce::getBodyQ(void) const
{
  const Vector6& speed = getAirSpeed();
  return speed(2);
}

const real_type&
AeroForce::getBodyR(void) const
{
  const Vector6& speed = getAirSpeed();
  return speed(3);
}

const real_type&
AeroForce::getMachNumber(void) const
{
  if (mDirtyMachNumber) {
    mMachNumber = norm(getMach());
    mDirtyMachNumber = false;
  }
  Log(ArtBody, Debug3) << "AeroForce::getMachNumber()"
                       << mMachNumber << endl;
  return mMachNumber;
}

const real_type&
AeroForce::getTrueSpeedUW(void) const
{
  if (mDirtyTrueSpeedUW) {
    const Vector6& speed = getAirSpeed();
    mTrueSpeedUW = sqrt(speed(4)*speed(4)+speed(6)*speed(6));
    mDirtyTrueSpeedUW = false;
  }
  Log(ArtBody, Debug3) << "AeroForce::getTrueSpeedUW()"
                       << mTrueSpeedUW << endl;
  return mTrueSpeedUW;
}

const real_type&
AeroForce::getWingSpanOver2Speed(void) const
{
  if (mDirtyWingSpanOver2Speed) {
    real_type Vt2 = 2*getTrueSpeed();
    if (fabs(Vt2) < Limits<real_type>::min())
      mWingSpanOver2Speed = 0;
    else
      mWingSpanOver2Speed = getWingSpan()/Vt2;
    mDirtyWingSpanOver2Speed = false;
  }
  return mWingSpanOver2Speed;
}

const real_type&
AeroForce::getCoordOver2Speed(void) const
{
  if (mDirtyCoordOver2Speed) {
    real_type Vt2 = 2*getTrueSpeed();
    if (fabs(Vt2) < Limits<real_type>::min())
      mCoordOver2Speed = 0;
    else
      mCoordOver2Speed = getCoord()/Vt2;
    mDirtyCoordOver2Speed = false;
  }
  return mCoordOver2Speed;
}

const real_type&
AeroForce::getHOverWingSpan(void) const
{
  if (mDirtyHOverWingSpan) {
    mHOverWingSpan = getAboveGroundLevel()/getWingSpan();
    mDirtyHOverWingSpan = false;
  }
  return mHOverWingSpan;
}

const real_type&
AeroForce::getAltitude(void) const
{
  if (mDirtyAltitude) {
    // FIXME temporary workaround
    if (!mEnvironment) {
      const_cast<AeroForce*>(this)->mEnvironment = getEnvironment();
    }
    // Get the altitude for the atmosphere.
    Geodetic geod = mEnvironment->getPlanet()->toGeod(getRefPosition());

    // Get the Athmosphere information at this position and the given time.
    mAltitude = geod.altitude;
    mDirtyAltitude = false;
  }
  Log(ArtBody, Debug3) << "AeroForce::getAltitude() " << mAltitude << endl;
  return mAltitude;
}

const real_type&
AeroForce::getAboveGroundLevel(void) const
{
  if (mDirtyAboveGroundLevel) {
    // Compute the intersection point with the ground plane in down direction
    Vector3 intersectPoint;
    if (getLocalGroundPlane().intersectLine(getPosition(), getUnitDown(),
                                            intersectPoint)) {
      mAboveGroundLevel = norm(intersectPoint);
    } else {
      // Hmm, no intersection? down must be parallel to the plane
      // FIXME, don't know what is best here
      mAboveGroundLevel = 1000;
    }
    mDirtyAboveGroundLevel = false;
  }
  Log(ArtBody, Debug3) << "AeroForce::getAboveGroundLevel() "
                       << mAboveGroundLevel << endl;
  return mAboveGroundLevel;
}

const real_type&
AeroForce::getPressure(void) const
{
  computeAtmosphere();
  Log(ArtBody, Debug3) << "AeroForce::getPressure() "
                       << mAtmos.pressure << endl;
  return mAtmos.pressure;
}

const real_type&
AeroForce::getDensity(void) const
{
  computeAtmosphere();
  Log(ArtBody, Debug3) << "AeroForce::getDensity() "
                       << mAtmos.density << endl;
  return mAtmos.density;
}

const real_type&
AeroForce::getSoundSpeed(void) const
{
  computeAtmosphere();
  Log(ArtBody, Debug3) << "AeroForce::getSoundSpeed() "
                       << mSoundSpeed << endl;
  return mSoundSpeed;
}

const real_type&
AeroForce::getTemperature(void) const
{
  computeAtmosphere();
  Log(ArtBody, Debug3) << "AeroForce::getTemperature() "
                       << mAtmos.temperature << endl;
  return mAtmos.temperature;
}

const real_type&
AeroForce::getPressureSeaLevel(void) const
{
  computeSLAtmosphere();
  return mSLAtmos.pressure;
}

const real_type&
AeroForce::getDensitySeaLevel(void) const
{
  computeSLAtmosphere();
  return mSLAtmos.density;
}

const real_type&
AeroForce::getSoundSpeedSeaLevel(void) const
{
  computeSLAtmosphere();
  return mSLSoundSpeed;
}

const real_type&
AeroForce::getTemperatureSeaLevel(void) const
{
  computeSLAtmosphere();
  return mSLAtmos.temperature;
}

const Vector3&
AeroForce::getUnitDown(void) const
{
  if (mDirtyUnitDown) {
    // Compute the geodetic unit down vector at our current position.
    // So we will need the orientation of the horizontal local frame at our
    // current position.
    Quaternion gcHL = getPlanet()->getGeocHLOrientation(getRefPosition());
    // Transform that unit down vector to the current frame.
    mUnitDown = mMountFrame->rotFromRef(gcHL.backTransform(Vector3::unit(3)));
    mDirtyUnitDown = false;
  }
  return mUnitDown;
}

const Plane&
AeroForce::getLocalGroundPlane(void) const
{
  if (mDirtyLocalGroundPlane) {
    // Transform the plane equation to the local frame.
    mLocalGroundPlane = mMountFrame->planeFromRef(mGroundVal.plane);
    mDirtyLocalGroundPlane = false;
  }
  return mLocalGroundPlane;
}

void
AeroForce::dirtyAll(void)
{
  // Dirty everything.
  mDirtyRefPosition = true;
  mDirtyUnitDown = true;
  mDirtyLocalGroundPlane = true;
  mDirtyAtmosphere = true;
  mDirtyAltitude = true;
  mDirtyAboveGroundLevel = true;
  mDirtyWingSpanOver2Speed = true;
  mDirtyCoordOver2Speed = true;
  mDirtyHOverWingSpan = true;
  mDirtySLAtmosphere = true;
  mDirtyAirSpeed = true;
  mDirtyMach = true;
  mDirtyMachNumber = true;
  mDirtyTrueSpeed = true;
  mDirtyTrueSpeedUW = true;
  mDirtyCalibratedAirSpeed = true;
  mDirtyEquivalentAirSpeed = true;
  mDirtyDynamicPressure = true;
  mDirtyAlpha = true;
  mDirtyAlphaDot = true;
  mDirtyBeta = true;
  mDirtyBetaDot = true;
}

void
AeroForce::computeAtmosphere(void) const
{
  if (mDirtyAtmosphere) {
    // Get the Athmosphere information at this position and the given time.
    mAtmos = getAtmosphere()->getData(getAltitude());
    mSoundSpeed = getAtmosphere()->getSoundSpeed(mAtmos.temperature);
    mDirtyAtmosphere = false;
  }
}

void
AeroForce::computeSLAtmosphere(void) const
{
  if (mDirtySLAtmosphere) {
    // Hmm, may be this does not need to be computed each time???
    mSLAtmos = getAtmosphere()->getData(0);
    mSLSoundSpeed = getAtmosphere()->getSoundSpeed(mSLAtmos.temperature);
    mDirtySLAtmosphere = false;
  }
}

void
AeroForce::computeCalEquAirspeed(void) const
{
  real_type p = getPressure();
  real_type psl = getPressureSeaLevel();
  real_type rhosl = getDensitySeaLevel();
  Vector3 mach = getMach();
  real_type qbar = getDynamicPressure();
  // Calibrated Airspeed
  real_type tube_press;
  if (mach(1) < 1) {   // Calculate total pressure assuming isentropic flow
    tube_press = p*pow((1 + 0.2*mach(1)*mach(1)), 3.5);
  } else {
    // Use Rayleigh pitot tube formula for normal shock in front of pitot tube
    real_type B = 5.76*mach(1)*mach(1)/(5.6*mach(1)*mach(1) - 0.8);
    real_type D = 0.4167*(2.8*mach(1)*mach(1) - 0.4);
    tube_press = p*pow(B, 3.5)*D;
  }
  real_type A = pow(((tube_press-p)/psl+1), 0.28571);
  if (mach(1) > 0) {
    mCalibratedAirSpeed = sqrt(7*psl/rhosl*(A-1));
    mEquivalentAirSpeed = sqrt(2*qbar/rhosl);
  } else {
    mCalibratedAirSpeed = 0;
    mEquivalentAirSpeed = 0;
  }
  
  mDirtyCalibratedAirSpeed = false;
  mDirtyEquivalentAirSpeed = false;
}

} // namespace OpenFDM
