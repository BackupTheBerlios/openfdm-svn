/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "RootJoint.h"

#include "ConstNodeVisitor.h"
#include "JointContext.h"
#include "MechanicLinkValue.h"
#include "NodeVisitor.h"

namespace OpenFDM {

class RootJoint::Context : public JointContext {
public:
  Context(const RootJoint* rootJoint, const Environment* environment,
          MechanicLinkValue* parentLinkValue, MechanicLinkValue* childLinkValue,
          PortValueList& portValueList) :
    JointContext(environment, parentLinkValue, childLinkValue, portValueList),
    mRootJoint(rootJoint)
  {}
  virtual ~Context() {}
  
  virtual const RootJoint& getNode() const
  { return *mRootJoint; }
  
  virtual void initDesignPosition()
  {
    mRootJoint->initDesignPosition(mPortValueList);
  }

  virtual void initVelocities(const /*Init*/Task& task)
  {
    mRootJoint->init(task, mDiscreteState, mContinousState, mPortValueList);
    mRootJoint->velocity(task, getEnvironment(), mContinousState, mPortValueList);
  }
  
  virtual void velocities(const Task& task)
  {
    mRootJoint->velocity(task, getEnvironment(), mContinousState, mPortValueList);
  }
  virtual void articulation(const Task& task)
  {
    mRootJoint->articulation(task, getEnvironment(), mContinousState, mPortValueList);
  }
  virtual void accelerations(const Task& task)
  {
    mRootJoint->acceleration(task, getEnvironment(), mContinousState, mPortValueList);
  }
  
  virtual void derivative(const Task&)
  {
    mRootJoint->derivative(getEnvironment(), mDiscreteState, mContinousState, mPortValueList,
                           mContinousStateDerivative);
  }
  
  virtual void update(const DiscreteTask&)
  { }

private:
  SharedPtr<const RootJoint> mRootJoint;
};
  
BEGIN_OPENFDM_OBJECT_DEF(RootJoint, Joint)
  END_OPENFDM_OBJECT_DEF

RootJoint::RootJoint(const std::string& name) :
  Joint(name)
{
}

RootJoint::~RootJoint()
{
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

JointContext*
RootJoint::newJointContext(const Environment* environment,
                           MechanicLinkValue* parentLinkValue,
                           MechanicLinkValue* childLinkValue,
                           PortValueList& portValueList) const
{
  SharedPtr<Context> context = new Context(this, environment, parentLinkValue,
                                           childLinkValue, portValueList);
  if (!context->allocStates()) {
    Log(Model, Warning) << "Could not alloc for model \""
                        << getName() << "\"" << endl;
    return false;
  }
  return context.release();
}

} // namespace OpenFDM
