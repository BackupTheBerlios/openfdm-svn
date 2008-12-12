/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "Interact.h"

#include "ConstNodeVisitor.h"
#include "MechanicContext.h"
#include "NodeVisitor.h"

namespace OpenFDM {

class Interact::Context : public MechanicContext {
public:
  Context(const Interact* interact, const Environment* environment) :
    MechanicContext(environment),
    mInteract(interact)
  {}
  virtual ~Context() {}

  virtual const Interact& getNode() const
  { return *mInteract; }

  virtual void initDesignPosition()
  {
    mInteract->initDesignPosition(mPortValueList);
  }

  virtual void initVelocities(const /*Init*/Task& task)
  {
    mInteract->init(task, mDiscreteState, mContinousState, mPortValueList);
    mInteract->velocity(task, getEnvironment(), mContinousState, mPortValueList);
  }

  virtual void velocities(const Task& task)
  {
    mInteract->velocity(task, getEnvironment(), mContinousState, mPortValueList);
  }
  virtual void articulation(const Task& task)
  {
    mInteract->articulation(task, getEnvironment(), mContinousState, mPortValueList);
  }
  virtual void accelerations(const Task& task)
  {
    mInteract->acceleration(task, getEnvironment(), mContinousState, mPortValueList);
  }

  virtual void derivative(const Task& task)
  {
    mInteract->derivative(task, getEnvironment(), mDiscreteState, mContinousState, mPortValueList,
                          mContinousStateDerivative);
  }
 
  virtual void update(const DiscreteTask&)
  { }

  bool alloc()
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
  SharedPtr<const Interact> mInteract;
};

BEGIN_OPENFDM_OBJECT_DEF(Interact, MechanicNode)
  END_OPENFDM_OBJECT_DEF

Interact::Interact(const std::string& name) :
  MechanicNode(name)
{
}

Interact::~Interact()
{
}

void
Interact::accept(NodeVisitor& visitor)
{
  visitor.handleNodePathAndApply(this);
}

void
Interact::accept(ConstNodeVisitor& visitor) const
{
  visitor.handleNodePathAndApply(this);
}

MechanicContext*
Interact::newMechanicContext(const Environment* environment,
                             PortValueList& portValueList) const
{
  SharedPtr<Context> context = new Context(this, environment);
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
  if (!context->alloc()) {
    Log(Model, Warning) << "Could not alloc for model \""
                        << getName() << "\"" << endl;
    return false;
  }
  return context.release();
}

} // namespace OpenFDM
