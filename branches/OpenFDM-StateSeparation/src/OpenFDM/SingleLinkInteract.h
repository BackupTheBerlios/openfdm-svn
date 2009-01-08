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
      mPortValueList(portValueList),
      mLink(portValueList.getPortValue(*interact->mMechanicLink))
    {
      OpenFDMAssert(mLink.isConnected());
    }
    virtual ~Context() {}
    
    virtual const SingleLinkInteract& getNode() const = 0;

    virtual void initDesignPosition()
    { mLink.setDesignPosition(getNode().getPosition()); }
  
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
    
    ParentLink& getLink()
    { return mLink; }

  protected:
    // PortValues
    PortValueList mPortValueList;
    
    // Continous States
    ContinousStateValueVector mContinousState;
    ContinousStateValueVector mContinousStateDerivative;
    // Discrete States
    DiscreteStateValueVector mDiscreteState;
    
  private:
    ParentLink mLink;
  };
  
  /// Set the position of the sensor in design coordinates
  void setPosition(const Vector3& position);
  /// Get the position of the sensor in design coordinates
  const Vector3& getPosition() const;

protected:
  SharedPtr<MechanicLinkInfo> mMechanicLink;
  Vector3 mPosition;
};

} // namespace OpenFDM

#endif
