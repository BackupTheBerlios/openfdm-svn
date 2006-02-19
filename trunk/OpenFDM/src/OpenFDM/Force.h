/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Force_H
#define OpenFDM_Force_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Frame.h"
#include "RigidBody.h"
#include "Interact.h"

namespace OpenFDM {

class Force : public Interact {
  OPENFDM_OBJECT(Force, Interact);
public:
  Force(const std::string& name, unsigned numParents);
  virtual ~Force(void);

};

class ExternalForce : public Force {
  OPENFDM_OBJECT(ExternalForce, Force);
public:
  ExternalForce(const std::string& name) :
    Force(name, 1),
    mForce(0, 0, 0, 0, 0, 0)
  {
    mMountFrame = new FreeFrame(name);
  }
  virtual ~ExternalForce(void) {}

  virtual void recheckTopology(void)
  {
    if (!getParentRigidBody(0))
      return;
  
    // check for the frames
    Frame* frame = getParentRigidBody(0)->getFrame();
    if (!frame)
      return;
    if (!mMountFrame->isDirectChildFrameOf(frame))
      frame->addChildFrame(mMountFrame);
  }

  // Needs to call applyForce once ...
  virtual void interactWith(RigidBody* rigidBody)
  {
    OpenFDMAssert(rigidBody->getFrame()->isDirectParentFrameOf(mMountFrame));
    rigidBody->applyForce(mMountFrame->forceToParent(mForce));
  }

  const Vector3& getPosition(void) const
  { return mMountFrame->getPosition(); }
  void setPosition(const Vector3& pos)
  { mMountFrame->setPosition(pos); }

  const Quaternion& getOrientation(void) const
  { return mMountFrame->getOrientation(); }
  void setOrientation(const Quaternion& pos)
  { mMountFrame->setOrientation(pos); }

  const Vector6& getForce(void) const
  { return mForce; }

protected:
  /** Sets the force contribution of this force element.
   * Sets the force contribution of this current force element to
   * the parent rigid body to force.
   */
  void setForce(const Vector6& force)
  { mForce = force; }

  SharedPtr<FreeFrame> mMountFrame;

private:
  Vector6 mForce;
};

class InternalForce : public Force {
  OPENFDM_OBJECT(InternalForce, Force);
public:
  InternalForce(const std::string& name) :
    Force(name, 2),
    mForce(0, 0, 0, 0, 0, 0)
  {
    mMountFrame[0] = new FreeFrame(name  + "<0>");
    mMountFrame[1] = new FreeFrame(name  + "<1>");
  }
  virtual ~InternalForce(void) {}

  virtual void recheckTopology(void)
  {
    if (!getParentRigidBody(0) || !getParentRigidBody(1))
      return;
  
    // check for the frames
    Frame* frame0 = getParentRigidBody(0)->getFrame();
    if (!frame0)
      return;
    if (!mMountFrame[0]->isDirectChildFrameOf(frame0))
      frame0->addChildFrame(mMountFrame[0]);
    Frame* frame1 = getParentRigidBody(1)->getFrame();
    if (!frame1)
      return;
    if (!mMountFrame[1]->isDirectChildFrameOf(frame1))
      frame1->addChildFrame(mMountFrame[1]);
  }

  virtual void interactWith(RigidBody* rigidBody)
  {
    // We assume that the given force is a positive force in the
    // frame 0's coordinates
    if (rigidBody->getFrame()->isDirectParentFrameOf(mMountFrame[0])) {
      Vector6 parentForce = mMountFrame[0]->forceToParent(mForce);
      rigidBody->applyForce(parentForce);
    } else if (rigidBody->getFrame()->isDirectParentFrameOf(mMountFrame[1])) {
      Rotation relOr = mMountFrame[0]->getRelOrientation(mMountFrame[1]);
      Vector6 force2(relOr.transform(mForce.getAngular()),
                     relOr.transform(mForce.getLinear()));
      Vector6 parentForce = mMountFrame[1]->forceToParent(force2);
      rigidBody->applyForce(-parentForce);
    }
  }

  const Vector3& getPosition0(void) const
  { return mMountFrame[0]->getPosition(); }
  void setPosition0(const Vector3& pos)
  { mMountFrame[0]->setPosition(pos); }

  const Quaternion& getOrientation0(void) const
  { return mMountFrame[0]->getOrientation(); }
  void setOrientation0(const Quaternion& pos)
  { mMountFrame[0]->setOrientation(pos); }

  const Vector3& getPosition1(void) const
  { return mMountFrame[1]->getPosition(); }
  void setPosition1(const Vector3& pos)
  { mMountFrame[1]->setPosition(pos); }

  const Quaternion& getOrientation1(void) const
  { return mMountFrame[1]->getOrientation(); }
  void setOrientation1(const Quaternion& pos)
  { mMountFrame[1]->setOrientation(pos); }

protected:
  SharedPtr<FreeFrame> mMountFrame[2];
  Vector6 mForce;
};

class LineForce : public InternalForce {
  OPENFDM_OBJECT(LineForce, InternalForce);
public:
  LineForce(const std::string& name) :
    InternalForce(name)
  {
    addOutputPort("relPos", this, &LineForce::getRelPos);
    addOutputPort("relVel", this, &LineForce::getRelVel);

    setNumInputPorts(1);
    setInputPortName(0, "force");
  }
  virtual ~LineForce(void) {}

  virtual bool init(void)
  {
    mForcePort = getInputPort("force")->toRealPortHandle();
    if (!mForcePort.isConnected()) {
      Log(Model, Error) << "Initialization of LineForce model \"" << getName()
                        << "\" failed: Input port \"" << getInputPortName(0)
                        << "\" is not connected!" << endl;
      return false;
    }
    return InternalForce::init();
  }
  virtual void output(const TaskInfo& taskInfo)
  {
    Vector3 dir;
    // FIXME: this is costly, must do something aprioriate with
    // models/ports
    Vector3 relPos = mMountFrame[0]->getRelPosition(mMountFrame[1]);
    // if we have really reached the zero position, we must have the full
    // speed in exactly the relPos direction.
    real_type nrmRelPos = norm(relPos);
    if (nrmRelPos < Limits<real_type>::min()) {
      Vector6 relVel6 = mMountFrame[0]->getRelVel(mMountFrame[1]);
      dir = normalize(relVel6.getLinear());
    } else
      dir = (1/nrmRelPos)*relPos;
    // Since we assume positive input forces to push the two attached
    // RigidBodies, we need that minus sign to negate the current position
    // offset
    mForce = Vector6(Vector3::zeros(), (-mForcePort.getRealValue())*dir);
  }

  const real_type& getRelPos(void) const
  {
    // FIXME: this is costly, must do something aprioriate with
    // models/ports
    mRelPos = norm(mMountFrame[0]->getRelPosition(mMountFrame[1]));
    return mRelPos;
  }
  const real_type& getRelVel(void) const
  {
    // FIXME: this is costly, must do something aprioriate with
    // models/ports
    Vector3 relPos = mMountFrame[0]->getRelPosition(mMountFrame[1]);
    Vector6 relVel6 = mMountFrame[0]->getRelVel(mMountFrame[1]);
    // if we have really reached the zero position, we must have the full
    // speed in exactly the relPos direction.
    real_type nrmRelPos = norm(relPos);
    if (nrmRelPos < Limits<real_type>::min())
      mRelVel = norm(relVel6.getLinear());
    else
      mRelVel = dot(relPos, relVel6.getLinear())/nrmRelPos;
    return mRelVel;
  }

private:
  mutable real_type mRelPos;
  mutable real_type mRelVel;

  /// The intput port which must provide the position
  RealPortHandle mForcePort;
};

} // namespace OpenFDM

#endif
