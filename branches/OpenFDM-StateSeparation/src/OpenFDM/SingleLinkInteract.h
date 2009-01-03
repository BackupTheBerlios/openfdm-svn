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
      mLinkRelPos(Vector3::zeros())
    {
      mMechanicLinkValue = portValueList.getPortValue(interact->mMechanicLink);
      OpenFDMAssert(mMechanicLinkValue);
    }
    virtual ~Context() {}
    
    virtual const SingleLinkInteract& getNode() const = 0;

    virtual void initDesignPosition()
    { mLinkRelPos = getNode().getPosition() - getLink().getDesignPosition(); }
  
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
    
    MechanicLinkValue& getLink() const
    { return *mMechanicLinkValue; }

    const Vector3& getLinkRelPos() const
    { return mLinkRelPos; }

    /// FIXME: Hmm, may be some kind of MechanicLinkHandle class that has a
    /// link value and these methods???
    void applyBodyForce(const Vector6& force)
    { mMechanicLinkValue->applyForce(mLinkRelPos, force); }
    void applyBodyForce(const Vector3& bodyPosition, const Vector6& force)
    { mMechanicLinkValue->applyForce(bodyPosition + mLinkRelPos, force); }
    void applyBodyForceAtLink(const Vector6& force)
    { mMechanicLinkValue->applyForce(force); }

    void applyBodyForce(const Vector3& force)
    { mMechanicLinkValue->applyForce(mLinkRelPos, force); }
    void applyBodyForce(const Vector3& bodyPosition, const Vector3& force)
    { mMechanicLinkValue->applyForce(bodyPosition + mLinkRelPos, force); }
    void applyBodyForceAtLink(const Vector3& force)
    { mMechanicLinkValue->applyForce(force); }

    void applyBodyTorque(const Vector3& torque)
    { mMechanicLinkValue->applyTorque(torque); }

  protected:
    // PortValues
    PortValueList mPortValueList;
    
    // Continous States
    ContinousStateValueVector mContinousState;
    ContinousStateValueVector mContinousStateDerivative;
    // Discrete States
    DiscreteStateValueVector mDiscreteState;
    
  private:
    SharedPtr<MechanicLinkValue> mMechanicLinkValue;
    Vector3 mLinkRelPos;
  };
  
  /// Set the position of the sensor in design coordinates
  void setPosition(const Vector3& position);
  /// Get the position of the sensor in design coordinates
  const Vector3& getPosition() const;

protected:
  MechanicLink mMechanicLink;

  Vector3 mPosition;
};

} // namespace OpenFDM

#endif
