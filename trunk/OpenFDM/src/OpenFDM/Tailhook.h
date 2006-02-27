/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Tailhook_H
#define OpenFDM_Tailhook_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Frame.h"
#include "Force.h"
#include "Ground.h"
#include "Environment.h"

namespace OpenFDM {

class Tailhook : public ExternalForce {
  OPENFDM_OBJECT(Tailhook, ExternalForce);
public:
  Tailhook(const std::string& name);
  virtual ~Tailhook(void);

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

  const real_type& getUpAngle(void) const
  { return mUpAngle; }
  void setUpAngle(const real_type& upAngle)
  { mUpAngle = upAngle; }

  const real_type& getDownAngle(void) const
  { return mDownAngle; }
  void setDownAngle(const real_type& downAngle)
  { mDownAngle = downAngle; }

private:
  void getGround(real_type t);
  bool computeWireFrame(real_type t, real_type& width);
  real_type computeCurrentTailhookAngle(void);

  real_type mLength;
  real_type mUpAngle;
  real_type mDownAngle;
  real_type mAngularVelocity;

  bool mHasWire;
  bool mFirstTime;

  /// Continous output of the launchbar angle
  real_type mAngle;
  /// discrete state of the laged tailhook position following the command
  /// the hooks position is immediately corrected for no ground intersection
  real_type mAngleCommand;

  /// The hooks position at the prevous step
  HookPosition mOldHookPosition;

  RealPortHandle mHookPositionPort;

  /// The frame where the catapult values are put in
  SharedPtr<FreeFrame> mWireFrame;

  GroundValues mGroundVal;
  SharedPtr<Environment> mEnvironment;
};

} // namespace OpenFDM

#endif
