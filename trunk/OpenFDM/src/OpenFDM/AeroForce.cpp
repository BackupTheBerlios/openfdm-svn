/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Frame.h"
#include "Force.h"
#include "Expression.h"
#include "Atmosphere.h"
#include "AeroForce.h"

namespace OpenFDM {

AeroForce::AeroForce(Environment* env, const std::string& name)
  : ExternalForce(name)
{
  mEnvironment = env;

  // Initialize all the expression nodes we will need.
  for (int i = 0; i < 6; ++i) {
    mStabilityAxisSummers[i] = new SumExpressionImpl;
    mBodyAxisSummers[i] = new SumExpressionImpl;
  }

  setPosition(Vector3::zeros());
  setOrientation(Quaternion::unit());

  mWingSpan = 0.0;
  mWingArea = 0.0;
  mCoord = 0.0;

  mDirtyRefPosition = true;
  mDirtyUnitDown = false;
  mDirtyLocalGroundPlane = true;
  mDirtyAtmosphere = true;
  mDirtyAltitude = true;
  mDirtyAboveGroundLevel = true;
  mDirtySLAtmosphere = true;
  mDirtyAirSpeed = true;
  mDirtyMach = true;
  mDirtyTrueSpeed = true;
  mDirtyCalibratedAirSpeed = true;
  mDirtyEquivalentAirSpeed = true;
  mDirtyDynamicPressure = true;
  mDirtyAlpha = true;
  mDirtyAlphaDot = true;
  mDirtyBeta = true;
  mDirtyBetaDot = true;

  addProperty("position",
              Property(this, &AeroForce::getPosition, &AeroForce::setPosition));
//   addProperty("orientation", Property(this, &AeroForce::getOrientation, &AeroForce::setOrientation));

  addProperty("wingSpan",
              Property(this, &AeroForce::getWingSpan, &AeroForce::setWingSpan));
  addProperty("wingArea",
              Property(this, &AeroForce::getWingArea, &AeroForce::setWingArea));
  addProperty("coord",
              Property(this, &AeroForce::getCoord, &AeroForce::setCoord));

  addProperty("altitude",
              Property(this, &AeroForce::getAltitude));
  addProperty("aboveGroundLevel",
              Property(this, &AeroForce::getAboveGroundLevel));

  addProperty("trueSpeed",
              Property(this, &AeroForce::getTrueSpeed));
  addProperty("dynamicPressure",
              Property(this, &AeroForce::getDynamicPressure));
  addProperty("alpha",
              Property(this, &AeroForce::getAlpha));
  addProperty("alphaDot",
              Property(this, &AeroForce::getAlphaDot));
  addProperty("beta",
              Property(this, &AeroForce::getBeta));
  addProperty("betaDot",
              Property(this, &AeroForce::getBetaDot));
  addProperty("mach",
              Property(this, &AeroForce::getMachNumber));
  addProperty("trueSpeedUW",
              Property(this, &AeroForce::getTrueSpeedUW));
  addProperty("wingSpanOver2Speed",
              Property(this, &AeroForce::getWingSpanOver2Speed));
  addProperty("coordOver2Speed",
              Property(this, &AeroForce::getCoordOver2Speed));
  addProperty("hOverWingSpan",
              Property(this, &AeroForce::getHOverWingSpan));

  addProperty("pressure",
              Property(this, &AeroForce::getPressure));
  addProperty("density",
              Property(this, &AeroForce::getDensity));
  addProperty("soundSpeed",
              Property(this, &AeroForce::getSoundSpeed));
  addProperty("temperature",
              Property(this, &AeroForce::getTemperature));

  addProperty("u",
              Property(this, &AeroForce::getBodyU));
  addProperty("v",
              Property(this, &AeroForce::getBodyV));
  addProperty("w",
              Property(this, &AeroForce::getBodyW));
  addProperty("p",
              Property(this, &AeroForce::getBodyP));
  addProperty("q",
              Property(this, &AeroForce::getBodyQ));
  addProperty("r",
              Property(this, &AeroForce::getBodyR));
}

AeroForce::~AeroForce(void)
{
}

void
AeroForce::setPosition(const Vector3& p)
{
  mPosition = p;
}

const Vector3&
AeroForce::getPosition(void) const
{
  return mPosition;
}

void
AeroForce::setOrientation(const Quaternion& o)
{
  mOrientation = o;
}

const Rotation&
AeroForce::getOrientation(void) const
{
  return mOrientation;
}

void
AeroForce::setWingSpan(const real_type& winSpan)
{
  mWingSpan = winSpan;
}

real_type
AeroForce::getWingSpan(void) const
{
  return mWingSpan;
}

void
AeroForce::setWingArea(const real_type& winArea)
{
  mWingArea = winArea;
}

real_type
AeroForce::getWingArea(void) const
{
  return mWingArea;
}

void
AeroForce::setCoord(const real_type& coord)
{
  mCoord = coord;
}

real_type
AeroForce::getCoord(void) const
{
  return mCoord;
}

const Vector3&
AeroForce::getRefPosition(void) const
{
  if (mDirtyRefPosition) {
    const Frame* frame = getParentFrame(0);
    OpenFDMAssert(frame);
    if (frame) {
      // Get the position in the earth centered coordinate frame.
      mRefPosition = frame->posToRef(getPosition());
      mDirtyRefPosition = false;
    }
  }
  return mRefPosition;
}

const Vector6&
AeroForce::getAirSpeed(void) const
{
  if (mDirtyAirSpeed) {
    const Frame* frame = getParentFrame(0);
    OpenFDMAssert(frame);
    if (frame) {
      // Get the position in the earth centered coordinate frame.
      Vector3 pos = frame->posToRef(getPosition());
      Vector3 windVel = mEnvironment->getWind()->getWindVel(pos);
      windVel = frame->rotFromRef(windVel);
      Vector6 sAirSpeed = Vector6(Vector3::zeros(), windVel)
        - frame->motionFromRef(Vector6::zeros());
      mAirSpeed = motionTo(getPosition(), getOrientation(), sAirSpeed);
      
      mDirtyAirSpeed = false;
    }
  }
  return mAirSpeed;
}

const Vector3&
AeroForce::getMach(void) const
{
  if (mDirtyMach) {
    mMach = (1/getSoundSpeed())*getAirSpeed().getLinear();
    mDirtyMach = false;
  }
  return mMach;
}

real_type
AeroForce::getTrueSpeed(void) const
{
  if (mDirtyTrueSpeed) {
    mTrueSpeed = norm(getAirSpeed().getLinear());
    mDirtyTrueSpeed = false;
  }
  return mTrueSpeed;
}

real_type
AeroForce::getEquivalentAirSpeed(void) const
{
  if (mDirtyEquivalentAirSpeed)
    computeCalEquAirspeed();
  return mEquivalentAirSpeed;
}

real_type
AeroForce::getCalibratedAirSpeed(void) const
{
  if (mDirtyCalibratedAirSpeed)
    computeCalEquAirspeed();
  return mCalibratedAirSpeed;
}

real_type
AeroForce::getDynamicPressure(void) const
{
  if (mDirtyDynamicPressure) {
    real_type V = getTrueSpeed();
    real_type rho = getDensity();
    mDynamicPressure = 0.5*rho*V*V;
  }
  return mDynamicPressure;
}

real_type
AeroForce::getAlpha(void) const
{
  if (mDirtyAlpha) {
    Vector3 V = getAirSpeed().getLinear();
    if (abs(V(iU)) < Limits<real_type>::min())
      mAlpha = 0;
    else
      mAlpha = atan2(V(iW), V(iU));
    mDirtyAlpha = false;
  }
  return mAlpha;
}

real_type
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
  return mAlphaDot;
}

real_type
AeroForce::getBeta(void) const
{
  if (mDirtyBeta) {
    real_type Vuw = getTrueSpeedUW();
    Vector3 V = getAirSpeed().getLinear();
    if (abs(Vuw) < Limits<real_type>::min())
      mBeta = 0;
    else
      mBeta = atan2(V(iV), Vuw);
    
    mDirtyBeta = false;
  }
  return mBeta;
}

real_type
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
  return mBetaDot;
}

real_type
AeroForce::getBodyU(void) const
{
  const Vector6& speed = getAirSpeed();
  return speed(4);
}

real_type
AeroForce::getBodyV(void) const
{
  const Vector6& speed = getAirSpeed();
  return speed(5);
}

real_type
AeroForce::getBodyW(void) const
{
  const Vector6& speed = getAirSpeed();
  return speed(6);
}

real_type
AeroForce::getBodyP(void) const
{
  const Vector6& speed = getAirSpeed();
  return speed(1);
}

real_type
AeroForce::getBodyQ(void) const
{
  const Vector6& speed = getAirSpeed();
  return speed(2);
}

real_type
AeroForce::getBodyR(void) const
{
  const Vector6& speed = getAirSpeed();
  return speed(3);
}

real_type
AeroForce::getMachNumber(void) const
{
  return getTrueSpeed()/getSoundSpeed();
}

real_type
AeroForce::getTrueSpeedUW(void) const
{
  const Vector6& speed = getAirSpeed();
  return sqrt(speed(4)*speed(4)+speed(6)*speed(6));
}

real_type
AeroForce::getWingSpanOver2Speed(void) const
{
  real_type Vt2 = 2*getTrueSpeed();
  if (abs(Vt2) < Limits<real_type>::min())
    return 0;
  else
    return getWingSpan()/Vt2;
}

real_type
AeroForce::getCoordOver2Speed(void) const
{
  real_type Vt2 = 2*getTrueSpeed();
  if (abs(Vt2) < Limits<real_type>::min())
    return 0;
  else
    return getCoord()/Vt2;
}

real_type
AeroForce::getHOverWingSpan(void) const
{
  return getAboveGroundLevel()/getWingSpan();
}

real_type
AeroForce::getAltitude(void) const
{
  if (mDirtyAltitude) {
    // Get the altitude for the atmosphere.
    Geodetic geod = mEnvironment->getPlanet()->toGeod(getRefPosition());

    // Get the Athmosphere information at this position and the given time.
    mAltitude = geod.altitude;
    mDirtyAltitude = false;
  }
  return mAltitude;
}

real_type
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
  return mAboveGroundLevel;
}

real_type
AeroForce::getPressure(void) const
{
  computeAtmosphere();
  return mAtmos.pressure;
}

real_type
AeroForce::getDensity(void) const
{
  computeAtmosphere();
  return mAtmos.density;
}

real_type
AeroForce::getSoundSpeed(void) const
{
  computeAtmosphere();
  return mAtmos.soundspeed;
}

real_type
AeroForce::getTemperature(void) const
{
  computeAtmosphere();
  return mAtmos.temperature;
}

real_type
AeroForce::getPressureSeaLevel(void) const
{
  computeSLAtmosphere();
  return mSLAtmos.pressure;
}

real_type
AeroForce::getDensitySeaLevel(void) const
{
  computeSLAtmosphere();
  return mSLAtmos.density;
}

real_type
AeroForce::getSoundSpeedSeaLevel(void) const
{
  computeSLAtmosphere();
  return mSLAtmos.soundspeed;
}

real_type
AeroForce::getTemperatureSeaLevel(void) const
{
  computeSLAtmosphere();
  return mSLAtmos.temperature;
}

const Vector3&
AeroForce::getUnitDown(void) const
{
  if (mDirtyUnitDown) {
    const Frame* frame = getParentFrame(0);
    OpenFDMAssert(frame);
    if (frame) {
      // Compute the geodetic unit down vector at our current position.
      // So we will need the orientation of the horizontal local frame at our
      // current position.
      Quaternion gcHL = getPlanet()->getGeocHLOrientation(getRefPosition());
      // Transform that unit down vector to the current frame.
      mUnitDown = frame->rotFromRef(gcHL.backTransform(Vector3::unit(3)));
      mDirtyUnitDown = false;
    }
  }
  return mUnitDown;
}

const Plane&
AeroForce::getLocalGroundPlane(void) const
{
  if (mDirtyLocalGroundPlane) {
    const Frame* frame = getParentFrame(0);
    OpenFDMAssert(frame);
    if (frame) {
      // Transform the plane equation to the local frame.
      mLocalGroundPlane = frame->planeFromRef(mGroundVal.plane);
      mDirtyLocalGroundPlane = false;
    }
  }
  return mLocalGroundPlane;
}

void
AeroForce::setConstantProperty(const std::string& pName, real_type value)
{
  Property prop(new ConstExpressionPropertyImpl<real_type>(value));
  addProperty(pName, prop);
}

void
AeroForce::removeConstantProperty(const std::string& pName)
{
  // FIXME check if it is a constant property
  removeProperty(pName);
}

void
AeroForce::setState(real_type t, const Vector& state, unsigned offset)
{
  // First determine the intersection depth with the ground.
  mGroundVal = mEnvironment->getGround()->getGroundPlane(t, getRefPosition());

  // Dirty everything.
  mDirtyRefPosition = true;
  mDirtyUnitDown = true;
  mDirtyLocalGroundPlane = true;
  mDirtyAtmosphere = true;
  mDirtyAltitude = true;
  mDirtyAboveGroundLevel = true;
  mDirtySLAtmosphere = true;
  mDirtyAirSpeed = true;
  mDirtyMach = true;
  mDirtyTrueSpeed = true;
  mDirtyCalibratedAirSpeed = true;
  mDirtyEquivalentAirSpeed = true;
  mDirtyDynamicPressure = true;
  mDirtyAlpha = true;
  mDirtyAlphaDot = true;
  mDirtyBeta = true;
  mDirtyBetaDot = true;
}

void
AeroForce::computeForce(void)
{
  // FIXME: they can be computed cheaper ...
  real_type ca = cos(getAlpha());
  real_type sa = sin(getAlpha());
  real_type cb = cos(getBeta());
  real_type sb = sin(getBeta());
  Matrix33 Ts2b(ca*cb, -ca*sb, -sa,
                   sb,     cb,   0,
                sa*cb, -sa*sb,  ca);

  // This is simple here. Just collect all summands ...
  Vector6 force = Vector6::zeros();
  for (int i = 0; i < 6; ++i)
    force(i+1) = mStabilityAxisSummers[i]->getValue();

  force.setAngular(Ts2b*force.getAngular());
  force.setLinear(Ts2b*force.getLinear());

  for (int i = 0; i < 6; ++i)
    force(i+1) += mBodyAxisSummers[i]->getValue();

  applyForce(forceFrom(mPosition, mOrientation, force));
}

void
AeroForce::computeAtmosphere(void) const
{
  if (mDirtyAtmosphere) {
    // Get the Athmosphere information at this position and the given time.
    mAtmos = getAtmosphere()->getData(getAltitude());
    mDirtyAtmosphere = false;
  }
}

void
AeroForce::computeSLAtmosphere(void) const
{
  if (mDirtySLAtmosphere) {
    // Hmm, may be this does not need to be computed each time???
    mSLAtmos = getAtmosphere()->getData(0);
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
