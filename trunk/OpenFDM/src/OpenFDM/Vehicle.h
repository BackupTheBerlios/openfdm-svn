/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Vehicle_H
#define OpenFDM_Vehicle_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Quaternion.h"
#include "Force.h"
#include "Frame.h"
#include "FreeJoint.h"
#include "RootFrame.h"
#include "MultiBodySystem.h"
#include "Planet.h"
#include "ODESolver.h"
#include "Environment.h"
#include "ModelGroup.h"
#include "System.h"

namespace OpenFDM {

class JSBReader;
class System;

/// FIXME: Derive that from System or something like that
class Vehicle :
    public Object {
public:
  Vehicle(void);
  virtual ~Vehicle(void);

  bool init(void);
  void output(void); /// FIXME??
  void update(real_type dt);

  bool trim(void);

  /** Set planet callback.
   */
  void setPlanet(Planet* p);

  /** Get planet callback.
   */
  const Planet* getPlanet(void) const
  { return mSystem->getEnvironment()->getPlanet(); }

  /** Set ground callback.
   */
  void setGround(Ground* g);

  /** Get ground callback.
   */
  const Ground* getGround(void) const
  { return mSystem->getEnvironment()->getGround(); }

  /** Set atmosphere callback.
   */
  void setAtmosphere(Atmosphere* g);

  /** Get atmosphere callback.
      FIXME ?? only callback??
   */
  const Atmosphere* getAtmosphere(void) const
  { return mSystem->getEnvironment()->getAtmosphere(); }

  /** Set wind callback.
   */
  void setWind(Wind* p);

  /** Get wind callback.
   */
  const Wind* getWind(void) const
  { return mSystem->getEnvironment()->getWind(); }

  /** Get the vehicle base node.
   */
  const FreeJoint* getFreeJoint(void) const
  { return mFreeJoint; }
  FreeJoint* getFreeJoint(void)
  { return mFreeJoint; }

  const ModelGroup* getModelGroup(void) const
  { return mModelGroup; }
  ModelGroup* getModelGroup(void)
  { return mModelGroup; }

  Vector3 getCartPosition(void) const;
  void setCartPosition(const Vector3& pos);
  Geodetic getGeodPosition(void) const;
  void setGeodPosition(const Geodetic& geod);
  Geocentric getGeocPosition(void) const;
  void setGeocPosition(const Geocentric& geoc);

  Quaternion getCartOrientation(void) const;
  void setCartOrientation(const Quaternion& o);
  Quaternion getGeocOrientation(void) const;
  void setGeocOrientation(const Quaternion& o);
  Quaternion getGeodOrientation(void) const;
  void setGeodOrientation(const Quaternion& o);

  // FIXME
  Vector6 getVelocity(void) const
  { return mTopBody->getFrame()->getRelVel(); }
  real_type getRadius(void) const
  { return 15; }
  real_type getTime(void) const
  { return mSystem->getTime(); }
  // FIXME:
  const RigidBody* getTopBody(void) const
  { return mTopBody; }
  RigidBody* getTopBody(void)
  { return mTopBody; }

  const System* getSystem(void) const
  { return mSystem; }
  System* getSystem(void)
  { return mSystem; }

  MultiBodySystem* getMultiBodySystem(void)
  { return mMultiBodySystem; }

private:
  SharedPtr<RigidBody> mTopBody;
  SharedPtr<FreeJoint> mFreeJoint;
  SharedPtr<RootFrame> mRootFrame;
  SharedPtr<MultiBodySystem> mMultiBodySystem;

  SharedPtr<ModelGroup> mModelGroup;

  SharedPtr<System> mSystem;
};

} // namespace OpenFDM

#endif
