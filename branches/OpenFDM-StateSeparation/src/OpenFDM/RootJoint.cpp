/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "RootJoint.h"

#include "ConstNodeVisitor.h"
#include "MechanicContext.h"
#include "NodeVisitor.h"

namespace OpenFDM {

class RootJoint::Context : public MechanicContext {
public:
  Context(const RootJoint* rootJoint) : mRootJoint(rootJoint) {}
  virtual ~Context() {}
  
  virtual const RootJoint& getNode() const
  { return *mRootJoint; }
  
  virtual bool alloc()
  { if (!allocStates()) return false; return mRootJoint->alloc(*this); }
  virtual void initVelocities(const /*Init*/Task& task)
  {
    mRootJoint->init(task, mDiscreteState, mContinousState, mPortValueList);
    mRootJoint->velocity(task, mContinousState, mPortValueList);
  }
  
  virtual void velocities(const Task& task)
  {
    mRootJoint->velocity(task, mContinousState, mPortValueList);
  }
  virtual void articulation(const Task& task)
  {
    mRootJoint->articulation(task, mContinousState, mPortValueList, hIh);
  }
  virtual void accelerations(const Task& task)
  {
    mRootJoint->acceleration(task, mContinousState, mPortValueList, hIh, velDot);
  }
  
  virtual void derivative(const Task&)
  {
    mRootJoint->derivative(mDiscreteState, mContinousState, mPortValueList,
                       velDot, mContinousStateDerivative);
  }
  
  virtual void update(const DiscreteTask&)
  { }
  
private:
  // Stores some values persistent accross velocity/articulation/acceleration
  Matrix hIh;
  Vector velDot;
  
  SharedPtr<const RootJoint> mRootJoint;
};
  
BEGIN_OPENFDM_OBJECT_DEF(RootJoint, Joint)
  DEF_OPENFDM_PROPERTY(Matrix, AngularBaseVelocity, Serialized)
  END_OPENFDM_OBJECT_DEF

RootJoint::RootJoint(const std::string& name) :
  Joint(name),
  mAngularBaseVelocity(Vector3::zeros())
{
}

RootJoint::~RootJoint()
{
}

MechanicContext*
RootJoint::newMechanicContext() const
{
  return new Context(this);
}

void
RootJoint::accept(NodeVisitor& visitor)
{
  visitor.handleNodePathAndApply(this);
}

void
RootJoint::accept(ConstNodeVisitor& visitor) const
{
  visitor.handleNodePathAndApply(this);
}

} // namespace OpenFDM
