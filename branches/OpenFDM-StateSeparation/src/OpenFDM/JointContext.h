/* -*-c++-*- OpenFDM - Copyright (C) 2007-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_JointContext_H
#define OpenFDM_JointContext_H

#include "Joint.h"
#include "MechanicContext.h"

namespace OpenFDM {

class JointContext : public MechanicContext {
public:
  JointContext(const Environment* environment,
               MechanicLinkValue* parentLinkValue,
               MechanicLinkValue* childLinkValue,
               PortValueList& portValueList) :
    MechanicContext(environment),
    mPortValueList(portValueList),
    mParentLink(parentLinkValue),
    mChildLink(childLinkValue)
  { }
  virtual ~JointContext() {}
    
  virtual const Joint& getNode() const = 0;
    
  bool allocStates()
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
  virtual const PortValue* getPortValue(const Port& portInfo) const
  {  return mPortValueList.getPortValue(portInfo); }
    
  ParentLink& getParentLink()
  { return mParentLink; }
  ChildLink& getChildLink()
  { return mChildLink; }

protected:
  // PortValues
  PortValueList mPortValueList;
    
  // Continous States
  ContinousStateValueVector mContinousState;
  ContinousStateValueVector mContinousStateDerivative;
  // Discrete States
  DiscreteStateValueVector mDiscreteState;

  ParentLink mParentLink;
  ChildLink mChildLink;
};

} // namespace OpenFDM

#endif
