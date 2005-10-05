/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Frame.h"
#include "RigidBody.h"
#include "Visitor.h"
#include "RootFrame.h"
#include "Mass.h"
#include "Force.h"

namespace OpenFDM {

class ForceComputationVisitor
  : public Visitor {
public:
  virtual void apply(Force& force)
  {
    force.computeForce();
  }
};

class ForwardDynamicsVisitor
  : public Visitor {
public:
  virtual void apply(RigidBody& body)
  {
    // Note the order. First compute the articulated values on each child.
    traverse(body);
    // Past that, do it on this current rigid body.
    body.computeArtValues();
  }
};

class AccelerationPropagationVisitor
  : public Visitor {
public:
  virtual void apply(Joint& joint)
  {
    joint.updateAccels();
  }
};

class SetStateVisitor
  : public Visitor {
public:
  SetStateVisitor(const Vector& state, real_type t)
    : mState(state), mTime(t), mOffset(0u)
  { }
  virtual void apply(MultiBodyModel& abNode)
  {
    OpenFDMAssert(mOffset + abNode.getNumContinousStates() <= mState.size());
    abNode.setState(mTime, mState, mOffset);
    mOffset += abNode.getNumContinousStates();
  }
private:
  Vector mState;
  unsigned mOffset;
  real_type mTime;
};

class GetStateVisitor
  : public ConstVisitor {
public:
  GetStateVisitor(unsigned size)
    : mState(size), mOffset(0u)
  { }
  virtual void apply(const MultiBodyModel& abNode)
  {
    OpenFDMAssert(mOffset + abNode.getNumContinousStates() <= mState.size());
    abNode.getState(mState, mOffset);
    mOffset += abNode.getNumContinousStates();
  }
  const Vector& getState(void) const
  { return mState; }
private:
  Vector mState;
  unsigned mOffset;
};

class GetStateDerivVisitor
  : public Visitor {
public:
  GetStateDerivVisitor(unsigned size)
    : mStateDeriv(size), mOffset(0u)
  { }
  virtual void apply(MultiBodyModel& abNode)
  {
    OpenFDMAssert(mOffset + abNode.getNumContinousStates() <= mStateDeriv.size());
    abNode.getStateDeriv(mStateDeriv, mOffset);
    mOffset += abNode.getNumContinousStates();
  }
  const Vector& getStateDeriv(void) const
  { return mStateDeriv; }
private:
  Vector mStateDeriv;
  unsigned mOffset;
};

class OutputVisitor
  : public Visitor {
public:
  OutputVisitor(void)
  { }
  virtual void apply(MultiBodyModel& abNode)
  {
    abNode.output();
  }
};

class UpdateVisitor
  : public Visitor {
public:
  UpdateVisitor(real_type dt) :
    mDt(dt)
  { }
  virtual void apply(MultiBodyModel& abNode)
  {
    abNode.update(mDt);
  }
private:
  real_type mDt;
};

class StateCountVisitor
  : public ConstVisitor {
public:
  StateCountVisitor(void)
    : mOffset(0u)
  { }
  virtual void apply(const MultiBodyModel& abNode)
  {
    mOffset += abNode.getNumContinousStates();
  }
  unsigned getStateCount(void) const
  { return mOffset; }
private:
  unsigned mOffset;
};

RootFrame::RootFrame(const std::string& name)
  : FreeFrame(name)
{
}

RootFrame::~RootFrame(void)
{
}


MultiBodySystem::~MultiBodySystem(void)
{
}

void
MultiBodySystem::setEvalState(real_type t, const Vector& state)
{
  // First we need to inject the current state into the tree of parts.
  setState(t, state, 0);

  // Compute the external and interaction forces.
  ForceComputationVisitor forceVisitor;
  mRootFrame->accept(forceVisitor);

  // Compute forward dynamics, that is the articulated forces and inertia.
  ForwardDynamicsVisitor fwdVisitor;
  mRootFrame->accept(fwdVisitor);

  // Then compute the articulated inertias and forces.
  AccelerationPropagationVisitor apVisitor;
  mRootFrame->accept(apVisitor);
}

void
MultiBodySystem::computeStateDeriv(real_type t, const Vector& state, Vector& deriv)
{
  setEvalState(t, state);

  // And finally extract the derivative vector from the tree.
  GetStateDerivVisitor gsdv(getNumContinousStates());
  mRootFrame->accept(gsdv);
  deriv = gsdv.getStateDeriv();
}

void
MultiBodySystem::setState(real_type t, const Vector& state, unsigned offset)
{
  SetStateVisitor ssv(state(Range(offset+1, offset+getNumContinousStates())), t);
  mRootFrame->accept(ssv);
}

void
MultiBodySystem::getState(Vector& state, unsigned offset) const
{
  GetStateVisitor gsv(getNumContinousStates());
  mRootFrame->accept(gsv);
  state(Range(offset+1, offset+getNumContinousStates())) = gsv.getState();
}

void
MultiBodySystem::getStateDeriv(Vector& stateDeriv, unsigned offset)
{
  OpenFDMAssert(offset == 0);

  // Compute the external and interaction forces.
  ForceComputationVisitor forceVisitor;
  mRootFrame->accept(forceVisitor);

  // Compute forward dynamics, that is the articulated forces and inertia.
  ForwardDynamicsVisitor fwdVisitor;
  mRootFrame->accept(fwdVisitor);

  // Then compute the articulated inertias and forces.
  AccelerationPropagationVisitor apVisitor;
  mRootFrame->accept(apVisitor);

  // And finally extract the derivative vector from the tree.
  GetStateDerivVisitor gsdv(getNumContinousStates());
  mRootFrame->accept(gsdv);
//   stateDeriv(Range(offset+1, offset+getNumContinousStates()))
//     = gsdv.getStateDeriv();
  stateDeriv
    = gsdv.getStateDeriv();
}

bool
MultiBodySystem::init(void)
{
  StateCountVisitor gsc;
  mRootFrame->accept(gsc);
  setNumContinousStates(gsc.getStateCount());
  return true;
}

void
MultiBodySystem::output(void)
{
  OutputVisitor ov;
  mRootFrame->accept(ov);
}

void
MultiBodySystem::update(real_type dt)
{
  UpdateVisitor uv(dt);
  mRootFrame->accept(uv);
}

} // namespace OpenFDM
