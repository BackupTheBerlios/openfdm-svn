/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Vehicle_H
#define OpenFDM_Vehicle_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Quaternion.h"
#include "Frame.h"
#include "FreeJoint.h"
#include "RootFrame.h"
#include "Planet.h"
#include "ODESolver.h"
#include "Environment.h"
#include "ModelGroup.h"
#include "System.h"

namespace OpenFDM {

class JSBReader;
class System;

class Vehicle :
    public Object {
public:
  Vehicle(void);
  virtual ~Vehicle(void);

  bool init(void);
  void output(void);
  void update(real_type dt);

  /** Set planet callback.
   */
  void setPlanet(Planet* p);

  /** Get planet callback.
   */
  const Planet* getPlanet(void) const
  { return mEnvironment->getPlanet(); }

  /** Set ground callback.
   */
  void setGround(Ground* g);

  /** Get ground callback.
   */
  const Ground* getGround(void) const
  { return mEnvironment->getGround(); }

  /** Set atmosphere callback.
   */
  void setAtmosphere(Atmosphere* g);

  /** Get atmosphere callback.
      FIXME ?? only callback??
   */
  const Atmosphere* getAtmosphere(void) const
  { return mEnvironment->getAtmosphere(); }

  /** Set wind callback.
   */
  void setWind(Wind* p);

  /** Get wind callback.
   */
  const Wind* getWind(void) const
  { return mEnvironment->getWind(); }

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
  { return mTopBody->getRelVel(); }
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

  void addRigidBody(RigidBody* rigidBody)
  { mRigidBodyIdMap[rigidBody->getName()] = rigidBody; }
  RigidBody* getRigidBody(const std::string& id)
  {
    if (0 < mRigidBodyIdMap.count(id))
      return mRigidBodyIdMap[id];
    return 0;
  }
  void addJoint(Joint* joint)
  { mJointIdMap[joint->getName()] = joint; }
  Joint* getJoint(const std::string& id)
  {
    if (0 < mJointIdMap.count(id))
      return mJointIdMap[id];
    return 0;
  }

private:
  // Environment ...
  shared_ptr<Environment> mEnvironment;

  shared_ptr<RigidBody> mTopBody;
  shared_ptr<FreeJoint> mFreeJoint;
  shared_ptr<RootFrame> mRootFrame;
  shared_ptr<MultiBodySystem> mMultiBodySystem;

  shared_ptr<ModelGroup> mModelGroup;

  shared_ptr<System> mSystem;

  std::map<std::string, shared_ptr<Frame> > mFrameIdMap;
  std::map<std::string, shared_ptr<Joint> > mJointIdMap;
  std::map<std::string, shared_ptr<RigidBody> > mRigidBodyIdMap;
  std::map<std::string, shared_ptr<Force> > mForceIdMap;

  // FIXME make a attach blub at id ...
  friend class JSBReader;
};

} // namespace OpenFDM

#endif
