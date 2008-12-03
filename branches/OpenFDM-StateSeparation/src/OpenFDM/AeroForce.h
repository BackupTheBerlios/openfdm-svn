/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
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
#include "Environment.h"

namespace OpenFDM {

class AeroForce : public ExternalForce {
  OPENFDM_OBJECT(AeroForce, ExternalForce);
public:
  AeroForce(const std::string&);
  virtual ~AeroForce(void);

  virtual bool init(void);
  virtual void output(const TaskInfo&);

  void setWingSpan(const real_type& winSpan);
  const real_type& getWingSpan(void) const;

  void setWingArea(const real_type& winArea);
  const real_type& getWingArea(void) const;

  void setCoord(const real_type& coord);
  const real_type& getCoord(void) const;

  const Vector6& getAirSpeed(void) const;
  const Vector3& getMach(void) const;
  const real_type& getTrueSpeed(void) const;
//   const real_type& getEquivalentAirSpeed(void) const;
//   const real_type& getCalibratedAirSpeed(void) const;
  const real_type& getDynamicPressure(void) const;
  const real_type& getAlpha(void) const;
  const real_type& getAlphaDot(void) const;
  const real_type& getBeta(void) const;
  const real_type& getBetaDot(void) const;

  const real_type& getBodyU(void) const;
  const real_type& getBodyV(void) const;
  const real_type& getBodyW(void) const;

  const real_type& getBodyP(void) const;
  const real_type& getBodyQ(void) const;
  const real_type& getBodyR(void) const;

  const real_type& getMachNumber(void) const;
  const real_type& getTrueSpeedUW(void) const;

  /// FIXME, may be just provide 1/(2Vt)???
  /// Or provide the whole set of nondimentionalized values ...
  /// May be better since we can throw out the singularities ...
  const real_type& getWingSpanOver2Speed(void) const;
  const real_type& getCoordOver2Speed(void) const;
  const real_type& getHOverWingSpan(void) const;

protected:
  virtual void setEnvironment(Environment* environment);

private:
  void dirtyAll(void);
  void computeAtmosphere(void) const;
  void computeSLAtmosphere(void) const;
  void computeCalEquAirspeed(void) const;

  RealPortHandle mStabilityAxisForce[3];
  RealPortHandle mBodyAxisTorque[3];

  const Atmosphere* getAtmosphere(void) const
  { return mEnvironment->getAtmosphere(); }
  const Planet* getPlanet(void) const
  { return mEnvironment->getPlanet(); }
  const Ground* getGround(void) const
  { return mEnvironment->getGround(); }
  const Wind* getWind(void) const
  { return mEnvironment->getWind(); }

  SharedPtr<const Environment> mEnvironment;

  real_type mWingSpan;
  real_type mWingArea;
  real_type mCoord;

  mutable bool mDirtyRefPosition:1;
  mutable bool mDirtyUnitDown:1;
  mutable bool mDirtyLocalGroundPlane:1;
  mutable bool mDirtyAltitude:1;
  mutable bool mDirtyAboveGroundLevel:1;
  mutable bool mDirtyWingSpanOver2Speed:1;
  mutable bool mDirtyCoordOver2Speed:1;
  mutable bool mDirtyHOverWingSpan:1;
  mutable bool mDirtyAtmosphere:1;
  mutable bool mDirtySLAtmosphere:1;
  mutable bool mDirtyAirSpeed:1;
  mutable bool mDirtyMach:1;
  mutable bool mDirtyMachNumber:1;
  mutable bool mDirtyTrueSpeed:1;
  mutable bool mDirtyTrueSpeedUW:1;
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
  mutable real_type mSoundSpeed;
  mutable AtmosphereData mSLAtmos;
  mutable real_type mSLSoundSpeed;
  mutable Vector6 mAirSpeed;
  mutable Vector3 mMach;
  mutable real_type mMachNumber;
  mutable real_type mAltitude;
  mutable real_type mAboveGroundLevel;
  mutable real_type mWingSpanOver2Speed;
  mutable real_type mCoordOver2Speed;
  mutable real_type mHOverWingSpan;
  mutable real_type mTrueSpeed;
  mutable real_type mTrueSpeedUW;
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
