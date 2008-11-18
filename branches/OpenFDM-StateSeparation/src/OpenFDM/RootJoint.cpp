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
  
  virtual void initDesignPosition()
  {
    mRootJoint->initDesignPosition(mPortValueList);
  }

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
    mRootJoint->articulation(task, mContinousState, mPortValueList);
  }
  virtual void accelerations(const Task& task)
  {
    mRootJoint->acceleration(task, mContinousState, mPortValueList);
  }
  
  virtual void derivative(const Task&)
  {
    mRootJoint->derivative(mDiscreteState, mContinousState, mPortValueList,
                           mContinousStateDerivative);
  }
  
  virtual void update(const DiscreteTask&)
  { }
  
private:
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
RootJoint::newMechanicContext(const MechanicLinkInfo* parentLink,
                              const MechanicLinkInfo* childLink,
                              PortValueList& portValueList) const
{
  SharedPtr<Context> context = new Context(this);
  for (unsigned i = 0; i < getNumPorts(); ++i) {
    PortValue* portValue = portValueList.getPortValue(i);
    if (!portValue) {
      Log(Model, Error) << "No port value given for model \"" << getName()
                        << "\" and port \"" << getPort(i)->getName()
                        << "\"" << endl;
      return false;
    }
    context->setPortValue(*getPort(i), portValue);
  }
  if (!context->allocStates()) {
    Log(Model, Warning) << "Could not alloc for model \""
                        << getName() << "\"" << endl;
    return false;
  }
  return context.release();
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
