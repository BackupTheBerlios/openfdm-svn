/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_AeroForce_H
#define OpenFDM_AeroForce_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Frame.h"
#include "Force.h"
#include "Atmosphere.h"
#include "Ground.h"
#include "Wind.h"
#include "Planet.h"
#include "Expression.h"
#include "Environment.h"

namespace OpenFDM {

class AeroForce
  : public ExternalForce {
public:
  enum ForceAxis {
    RollAxis = 1,
    PitchAxis = 2,
    YawAxis = 3,
    DragAxis = 4,
    SideAxis = 5,
    LiftAxis = 6
  };
  AeroForce(Environment* env, const std::string&);
  virtual ~AeroForce(void);

  void setPosition(const Vector3& p);
  const Vector3& getPosition(void) const;

  void setOrientation(const Quaternion& o);
  const Rotation& getOrientation(void) const;

  void setWingSpan(const real_type& winSpan);
  real_type getWingSpan(void) const;

  void setWingArea(const real_type& winArea);
  real_type getWingArea(void) const;

  void setCoord(const real_type& coord);
  real_type getCoord(void) const;

  const Vector3& getRefPosition(void) const;
  const Vector6& getAirSpeed(void) const;
  const Vector3& getMach(void) const;
  real_type getTrueSpeed(void) const;
  real_type getEquivalentAirSpeed(void) const;
  real_type getCalibratedAirSpeed(void) const;
  real_type getDynamicPressure(void) const;
  real_type getAlpha(void) const;
  real_type getAlphaDot(void) const;
  real_type getBeta(void) const;
  real_type getBetaDot(void) const;

  real_type getBodyU(void) const;
  real_type getBodyV(void) const;
  real_type getBodyW(void) const;

  real_type getBodyP(void) const;
  real_type getBodyQ(void) const;
  real_type getBodyR(void) const;

  real_type getMachNumber(void) const;
  real_type getTrueSpeedUW(void) const;

  /// FIXME, may be just provide 1/(2Vt)???
  /// Or provide the whole set of nondimentionalized values ...
  /// May be better since we can throw out the singularities ...
  real_type getWingSpanOver2Speed(void) const;
  real_type getCoordOver2Speed(void) const;
  real_type getHOverWingSpan(void) const;

  real_type getAltitude(void) const;
  real_type getAboveGroundLevel(void) const;

  real_type getPressure(void) const;
  real_type getDensity(void) const;
  real_type getSoundSpeed(void) const;
  real_type getTemperature(void) const;


  real_type getPressureSeaLevel(void) const;
  real_type getDensitySeaLevel(void) const;
  real_type getSoundSpeedSeaLevel(void) const;
  real_type getTemperatureSeaLevel(void) const;

  const Vector3& getUnitDown(void) const;
  const Plane& getLocalGroundPlane(void) const;

  void setConstantProperty(const std::string& pName, real_type value);
  void removeConstantProperty(const std::string& pName);

  void addStabilityAxisSummand(ForceAxis axis, const RealProperty& prop)
  {
    mStabilityAxisSummers[axis-1]->addInputProperty(prop);
  }
  void addStabilityAxisSummand(ForceAxis axis, const Property& prop)
  {
    mStabilityAxisSummers[axis-1]->addInputProperty(prop);
  }
  void addBodyAxisSummand(ForceAxis axis, const RealProperty& prop)
  {
    mBodyAxisSummers[axis-1]->addInputProperty(prop);
  }
  void addBodyAxisSummand(ForceAxis axis, const Property& prop)
  {
    mBodyAxisSummers[axis-1]->addInputProperty(prop);
  }

  virtual void
  setState(real_type t, const Vector& state, unsigned offset);

  // The interface to the mechanical system.
  virtual void computeForce(void);

private:
  void computeAtmosphere(void) const;
  void computeSLAtmosphere(void) const;
  void computeCalEquAirspeed(void) const;

  shared_ptr<SumExpressionImpl> mStabilityAxisSummers[6];
  shared_ptr<SumExpressionImpl> mBodyAxisSummers[6];

  const Atmosphere* getAtmosphere(void) const
  { return mEnvironment->getAtmosphere(); }
  const Planet* getPlanet(void) const
  { return mEnvironment->getPlanet(); }

  shared_ptr<Environment> mEnvironment;

  Vector3 mPosition;
  Rotation mOrientation;

  real_type mWingSpan;
  real_type mWingArea;
  real_type mCoord;

  mutable bool mDirtyRefPosition:1;
  mutable bool mDirtyUnitDown:1;
  mutable bool mDirtyLocalGroundPlane:1;
  mutable bool mDirtyAltitude:1;
  mutable bool mDirtyAboveGroundLevel:1;
  mutable bool mDirtyAtmosphere:1;
  mutable bool mDirtySLAtmosphere:1;
  mutable bool mDirtyAirSpeed:1;
  mutable bool mDirtyMach:1;
  mutable bool mDirtyTrueSpeed:1;
  mutable bool mDirtyCalibratedAirSpeed:1;
  mutable bool mDirtyEquivalentAirSpeed:1;
  mutable bool mDirtyDynamicPressure:1;
  mutable bool mDirtyAlpha:1;
  mutable bool mDirtyAlphaDot:1;
  mutable bool mDirtyBeta:1;
  mutable bool mDirtyBetaDot:1;
  mutable Vector3 mRefPosition;
  mutable Vector3 mUnitDown;
  mutable Plane mLocalGroundPlane;
  mutable AtmosphereData mAtmos;
  mutable AtmosphereData mSLAtmos;
  mutable Vector6 mAirSpeed;
  mutable Vector3 mMach;
  mutable real_type mAltitude;
  mutable real_type mAboveGroundLevel;
  mutable real_type mTrueSpeed;
  mutable real_type mCalibratedAirSpeed;
  mutable real_type mEquivalentAirSpeed;
  mutable real_type mDynamicPressure;
  mutable real_type mAlpha;
  mutable real_type mAlphaDot;
  mutable real_type mBeta;
  mutable real_type mBetaDot;

  GroundValues mGroundVal;
};

} // namespace OpenFDM

#endif
