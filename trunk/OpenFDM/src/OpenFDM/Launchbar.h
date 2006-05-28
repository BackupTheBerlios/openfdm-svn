/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Launchbar_H
#define OpenFDM_Launchbar_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Frame.h"
#include "Force.h"
#include "Ground.h"
#include "Environment.h"

namespace OpenFDM {

class Launchbar : public ExternalForce {
  OPENFDM_OBJECT(Launchbar, ExternalForce);
public:
  Launchbar(const std::string& name);
  virtual ~Launchbar(void);

  virtual bool init(void);
  virtual void output(const TaskInfo&);
  virtual void update(const TaskInfo&);

  virtual void setDiscreteState(const StateStream& state);
  virtual void getDiscreteState(StateStream& state) const;

  const real_type& getAngle(void) const
  { return mAngle; }

  const real_type& getLength(void) const
  { return mLength; }
  void setLength(const real_type& length)
  { mLength = length; }

  const real_type& getHoldbackLength(void) const
  { return mHoldbackLength; }
  void setHoldbackLength(const real_type& length)
  { mHoldbackLength = length; }

  const Vector3& getHoldbackMount(void) const
  { return mHoldbackMount; }
  void setHoldbackMount(const Vector3& mount)
  { mHoldbackMount = mount; }

  const real_type& getUpAngle(void) const
  { return mUpAngle; }
  void setUpAngle(const real_type& upAngle)
  { mUpAngle = upAngle; }

  const real_type& getDownAngle(void) const
  { return mDownAngle; }
  void setDownAngle(const real_type& downAngle)
  { mDownAngle = downAngle; }

  const real_type& getLaunchForce(void) const
  { return mLaunchForce; }
  void setLaunchForce(const real_type& launchForce)
  { mLaunchForce = launchForce; }

protected:
  virtual void setEnvironment(Environment* environment);

private:
  void getGround(real_type t);
  bool computeCatFrame(real_type t, real_type& catLen);
  real_type computeCurrentLaunchbarAngle(void);

  enum State {
    Unmounted,
    Mounted,
    Launching
  };

  real_type mLength;
  real_type mHoldbackLength;
  // The mount point of the holdback at the strut
  Vector3 mHoldbackMount;
  real_type mUpAngle;
  real_type mDownAngle;
  real_type mAngularVelocity;
  // The launch force
  real_type mLaunchForce;

  /// Continous output of the launchbar angle
  real_type mAngle;
  /// discrete state of the laged launchbar position following the command
  real_type mAngleCommand;

  real_type mPosOnCat;
  State mState;

  RealPortHandle mTryMountPort;
  RealPortHandle mLaunchCommandPort;

  /// The frame where the catapult values are put in
  SharedPtr<FreeFrame> mCatFrame;

  GroundValues mGroundVal;
  SharedPtr<const Environment> mEnvironment;
};

} // namespace OpenFDM

#endif
