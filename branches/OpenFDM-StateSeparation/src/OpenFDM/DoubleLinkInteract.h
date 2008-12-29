/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_DoubleLinkInteract_H
#define OpenFDM_DoubleLinkInteract_H

#include <string>
#include "Interact.h"
#include "Environment.h"
#include "MechanicContext.h"

namespace OpenFDM {

class DoubleLinkInteract : public Interact {
  OPENFDM_OBJECT(DoubleLinkInteract, Interact);
public:
  DoubleLinkInteract(const std::string& name);
  virtual ~DoubleLinkInteract();

  class Context : public MechanicContext {
  public:
    Context(const DoubleLinkInteract* interact, const Environment* environment,
            PortValueList& portValueList) :
      MechanicContext(environment),
      mInteract(interact),
      mPortValueList(portValueList)
    {
      mMechanicLinkValue0 = portValueList.getPortValue(interact->mMechanicLink0.getPortIndex())->toMechanicLinkValue();
      OpenFDMAssert(mMechanicLinkValue0);
      mMechanicLinkValue1 = portValueList.getPortValue(interact->mMechanicLink1.getPortIndex())->toMechanicLinkValue();
      OpenFDMAssert(mMechanicLinkValue1);
    }
    virtual ~Context() {}
    
    virtual const DoubleLinkInteract& getNode() const
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
    
    MechanicLinkValue& getMechanicLinkValue0() const
    { return *mMechanicLinkValue0; }
    MechanicLinkValue& getMechanicLinkValue1() const
    { return *mMechanicLinkValue1; }

  protected:
    // PortValues
    PortValueList mPortValueList;
    
    // Continous States
    ContinousStateValueVector mContinousState;
    ContinousStateValueVector mContinousStateDerivative;
    // Discrete States
    DiscreteStateValueVector mDiscreteState;
    
  private:
    SharedPtr<const DoubleLinkInteract> mInteract;
    SharedPtr<MechanicLinkValue> mMechanicLinkValue0;
    SharedPtr<MechanicLinkValue> mMechanicLinkValue1;
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
  MechanicLink mMechanicLink0;
  MechanicLink mMechanicLink1;
};

} // namespace OpenFDM

#endif
