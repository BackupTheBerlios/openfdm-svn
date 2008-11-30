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

  virtual bool allocStates()
  {
    unsigned numContinousStates = getNode().getNumContinousStateValues();
    for (unsigned i = 0; i < numContinousStates; ++i) {
      const ContinousStateInfo* continousStateInfo;
      continousStateInfo = getNode().getContinousStateInfo(i);
      mContinousState.setValue(*continousStateInfo, *this);
      mContinousStateDerivative.setValue(*continousStateInfo, *this);
    }
    unsigned numDiscreteStates = getNode().getNumDiscreteStateValues();
    for (unsigned i = 0; i < numDiscreteStates; ++i) {
      const StateInfo* stateInfo;
      stateInfo = getNode().getDiscreteStateInfo(i);
      mDiscreteState.setValue(*stateInfo, *this);
    }
    return true;
  }
  
  virtual ContinousStateValue* getStateValue(const ContinousStateInfo& info)
  { return mContinousState.getValue(info); }
  virtual ContinousStateValue* getStateDerivative(const ContinousStateInfo& info)
  { return mContinousStateDerivative.getValue(info); }
  
  /// Set port value for the given port.
  virtual const PortValue* getPortValue(const PortInfo& portInfo) const
  {  return mPortValueList.getPortValue(portInfo); }
  void setPortValue(const PortInfo& portInfo, PortValue* portValue)
  { mPortValueList.setPortValue(portInfo.getIndex(), portValue); }
  
  
protected:
  // PortValues
  PortValueList mPortValueList;
  
  // Continous States
  ContinousStateValueVector mContinousState;
  ContinousStateValueVector mContinousStateDerivative;
  // Discrete States
  DiscreteStateValueVector mDiscreteState;
  
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

MechanicContext*
RootJoint::newMechanicContext(const Environment* environment,
                              const MechanicLinkInfo* parentLink,
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

    if (portValue->toMechanicLinkValue()) {
      portValue->toMechanicLinkValue()->setEnvironment(environment);
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
