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
      mPortValueList(portValueList)
    {
      mMechanicLinkValue0 = portValueList.getPortValue(*interact->mMechanicLinkInfo0);
      OpenFDMAssert(mMechanicLinkValue0);
      mMechanicLinkValue1 = portValueList.getPortValue(*interact->mMechanicLinkInfo1);
      OpenFDMAssert(mMechanicLinkValue1);
    }
    virtual ~Context() {}

    virtual const DoubleLinkInteract& getNode() const = 0;

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
    
    MechanicLinkValue& getLink0() const
    { return *mMechanicLinkValue0; }
    MechanicLinkValue& getLink1() const
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
    SharedPtr<MechanicLinkValue> mMechanicLinkValue0;
    SharedPtr<MechanicLinkValue> mMechanicLinkValue1;
  };
  
protected:
  SharedPtr<MechanicLinkInfo> mMechanicLinkInfo0;
  SharedPtr<MechanicLinkInfo> mMechanicLinkInfo1;
};

} // namespace OpenFDM

#endif
