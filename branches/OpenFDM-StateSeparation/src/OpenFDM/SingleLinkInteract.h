/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_SingleLinkInteract_H
#define OpenFDM_SingleLinkInteract_H

#include <string>
#include "Interact.h"
#include "Environment.h"
#include "MechanicContext.h"

namespace OpenFDM {

class ConstNodeVisitor;
class ContinousStateValueVector;
class DiscreteStateValueVector;
class NodeVisitor;
class PortValueList;
class Task;

class SingleLinkInteract : public Interact {
  OPENFDM_OBJECT(SingleLinkInteract, Interact);
public:
  SingleLinkInteract(const std::string& name);
  virtual ~SingleLinkInteract();

  class Context : public MechanicContext {
  public:
    Context(const SingleLinkInteract* interact, const Environment* environment,
            PortValueList& portValueList) :
      MechanicContext(environment),
      mInteract(interact),
      mPortValueList(portValueList)
    {
      mMechanicLinkValue = portValueList.getPortValue(interact->mMechanicLink.getPortIndex())->toMechanicLinkValue();
      OpenFDMAssert(mMechanicLinkValue);
    }
    virtual ~Context() {}
    
    virtual const SingleLinkInteract& getNode() const
    { return *mInteract; }
    
    virtual void initDesignPosition()
    {
      mInteract->initDesignPosition(mPortValueList);
    }
    
    virtual void init(const /*Init*/Task& task)
    {
      mInteract->init(task, mDiscreteState, mContinousState, mPortValueList);
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
    
    MechanicLinkValue& getMechanicLinkValue() const
    { return *mMechanicLinkValue; }

  protected:
    // PortValues
    PortValueList mPortValueList;
    
    // Continous States
    ContinousStateValueVector mContinousState;
    ContinousStateValueVector mContinousStateDerivative;
    // Discrete States
    DiscreteStateValueVector mDiscreteState;
    
  private:
    SharedPtr<const SingleLinkInteract> mInteract;
    SharedPtr<MechanicLinkValue> mMechanicLinkValue;
  };
  

  virtual MechanicContext* newMechanicContext(const Environment* environment,
                                              PortValueList& portValueList) const
  {
    SharedPtr<Context> context = new Context(this, environment, portValueList);
    if (!context->alloc()) {
      Log(Model, Warning) << "Could not alloc for model \""
                          << getName() << "\"" << endl;
      return 0;
    }
    return context.release();
  }
  
  virtual void init(const Task&, DiscreteStateValueVector&,
                    ContinousStateValueVector&, const PortValueList&) const
  { }
  virtual void initDesignPosition(PortValueList&) const = 0;
  virtual void velocity(const Task&, const Environment& environment,
                        const ContinousStateValueVector&, PortValueList&) const
  { }
  virtual void articulation(const Task&, const Environment& environment,
                            const ContinousStateValueVector&,
                            PortValueList&) const
  { }
  virtual void acceleration(const Task&, const Environment& environment,
                            const ContinousStateValueVector&,
                            PortValueList&) const
  { }
  virtual void derivative(const Task&, const Environment& environment,
                          const DiscreteStateValueVector&,
                          const ContinousStateValueVector&,
                          const PortValueList&,
                          ContinousStateValueVector&) const
  { }

protected:
  MechanicLink mMechanicLink;
};

} // namespace OpenFDM

#endif
