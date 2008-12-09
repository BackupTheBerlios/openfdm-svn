/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_CartesianJoint_H
#define OpenFDM_CartesianJoint_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "MatrixStateInfo.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Joint.h"
#include "MechanicContext.h"

namespace OpenFDM {

template<unsigned n>
class CartesianJoint : public Joint {
protected:
  class Context;

public:
  typedef LinAlg::Vector<real_type,n> VectorN;
  typedef LinAlg::Matrix<real_type,6,n> Matrix6N;
  typedef LinAlg::Matrix<real_type,n,n> MatrixNN;
  typedef LinAlg::MatrixFactors<real_type,n,n,LinAlg::LUTag> MatrixFactorsNN;

  // Each Cartesian joint has a position that has some kind of invariance
  const Vector3& getPosition() const
  { return mPosition; }
  void setPosition(const Vector3& position)
  { mPosition = position; }

  virtual MechanicContext*
  newMechanicContext(const Environment* environment,
                     const MechanicLinkInfo* parentLink,
                     const MechanicLinkInfo* childLink,
                     PortValueList& portValueList) const
  {
    if (!parentLink) {
      Log(Model, Error) << "Parent link is not set while creating context "
                        << "for model \"" << getName() << "\"" << endl;
      return 0;
    }
    MechanicLinkValue* parentLinkValue;
    parentLinkValue = portValueList.getPortValue(*parentLink);
    if (!parentLinkValue)
      return 0;
    if (!childLink) {
      Log(Model, Error) << "Child link is not set while creating context "
                        << "for model \"" << getName() << "\"" << endl;
      return 0;
    }
    MechanicLinkValue* childLinkValue;
    childLinkValue = portValueList.getPortValue(*childLink);
    if (!childLinkValue)
      return 0;

    // Now propagate the root dependent data ...
    OpenFDMAssert(environment == parentLinkValue->getEnvironment());
    childLinkValue->setEnvironment(environment);

    SharedPtr<Context> context;
    context = new Context(this, parentLinkValue, childLinkValue);
    for (unsigned i = 0; i < getNumPorts(); ++i) {
      PortValue* portValue = portValueList.getPortValue(i);
      if (!portValue) {
        Log(Model, Error) << "No port value given for model \"" << getName()
                          << "\" and port \"" << getPort(i)->getName()
                          << "\"" << endl;
        return 0;
      }
      context->setPortValue(*getPort(i), portValue);
    }
    if (!context->alloc()) {
      Log(Model, Warning) << "Could not alloc for model \""
                          << getName() << "\"" << endl;
      return 0;
    }
    return context.release();
  }

protected:
  CartesianJoint(const std::string& name) :
    Joint(name),
    mParentLink(newMechanicLink("link0")),
    mChildLink(newMechanicLink("link1")),
    mPosition(0, 0, 0)
  { }
  virtual ~CartesianJoint(void)
  { }

  virtual void init(const Task&, DiscreteStateValueVector&,
                    ContinousStateValueVector&, const PortValueList&) const
  { }
  virtual Matrix6N getJointMatrix() const = 0;

  virtual void velocity(const Task& task, Context& context,
                        const ContinousStateValueVector& states,
                        PortValueList& portValues) const = 0;
  virtual void articulation(const Task& task, Context& context,
                            const ContinousStateValueVector& states,
                            PortValueList& portValues) const = 0;
  virtual void acceleration(const Task& task, Context& context,
                            const ContinousStateValueVector& states,
                            PortValueList& portValues) const = 0;
  virtual void derivative(const Task& task, Context& context,
                          const ContinousStateValueVector&,
                          const PortValueList& portValues,
                          ContinousStateValueVector&) const = 0;

  class Context : public MechanicContext {
  public:
    Context(const CartesianJoint* cartesianJoint, MechanicLinkValue* parentLink,
            MechanicLinkValue* childLink) :
      mCartesianJoint(cartesianJoint),
      mParentLink(parentLink),
      mChildLink(childLink)
    { }
    virtual ~Context() {}
    
    virtual const CartesianJoint& getNode() const
    { return *mCartesianJoint; }
    
    virtual void initDesignPosition()
    {
      Vector3 jointPosition = mCartesianJoint->getPosition();
      mRelativePosition = jointPosition - mParentLink->getDesignPosition();
      mChildLink->setDesignPosition(jointPosition);

      mJointMatrix = mCartesianJoint->getJointMatrix();
    }

    bool alloc()
    {
      return allocStates();
    }
    virtual void initVelocities(const /*Init*/Task& task)
    {
      mCartesianJoint->init(task, mDiscreteState,
                            mContinousState, mPortValueList);
      mCartesianJoint->velocity(task, *this, mContinousState, mPortValueList);
    }
    
    virtual void velocities(const Task& task)
    {
      mCartesianJoint->velocity(task, *this, mContinousState, mPortValueList);
    }
    virtual void articulation(const Task& task)
    {
      mCartesianJoint->articulation(task, *this, mContinousState,
                                    mPortValueList);
    }
    virtual void accelerations(const Task& task)
    {
      mCartesianJoint->acceleration(task, *this, mContinousState,
                                    mPortValueList);
    }
    
    virtual void derivative(const Task& task)
    {
      mCartesianJoint->derivative(task, *this, mContinousState,
                                  mPortValueList, mContinousStateDerivative);
    }
    
    virtual void update(const DiscreteTask&)
    { }

    void setPosAndVel(const Vector3& position, const Quaternion& orientation,
                      const VectorN& velocity)
    {
      mChildLink->setPosAndVel(*mParentLink, mRelativePosition + position,
                               orientation, mJointMatrix*velocity);
    }

    /** Compute the articulation step for a given joint force.
     *  Use this for usual joints.
     */
    void applyJointForce(const VectorN& jointForce)
    {
      // The formulas conform to Roy Featherstones book eqn (6.37), (6.38)

      mJointForce = jointForce;
    
      // Store the outboard values since we will need them later in velocity
      // derivative computations
      SpatialInertia I = mChildLink->getInertia();
      
      // Compute the projection to the joint coordinate space
      Matrix6N Ih = I*mJointMatrix;
      hIh = trans(mJointMatrix)*Ih;

      // Note that the momentum of the local mass is already included in the
      // child links force due the the mass model ...
      pAlpha = mChildLink->getForce() + I*mChildLink->getFrame().getHdot();
      
      if (hIh.singular()) {
        Log(ArtBody,Error) << "Detected singular mass matrix for "
                           << "CartesianJoint \"" << mCartesianJoint->getName()
                           << "\": Fix your model!" << endl;
        return;
      }
      
      // Project away the directions handled with this current joint
      Vector6 force = pAlpha;
      force -= Ih*hIh.solve(trans(mJointMatrix)*pAlpha - jointForce);
      I -= SpatialInertia(Ih*hIh.solve(trans(Ih)));
      
      // Transform to parent link's coordinates and apply to the parent link
      mParentLink->applyForce(mChildLink->getFrame().forceToParent(force));
      mParentLink->applyInertia(mChildLink->getFrame().inertiaToParent(I));
    }

    /** Compute the acceleration step for a given joint force.
     *  Use this for usual joints.
     */
    void accelerateDueToForce()
    {
      Vector6 parentSpAccel
        = mChildLink->getFrame().motionFromParent(mParentLink->getFrame().getSpAccel());
    
      Vector6 f = mChildLink->getInertia()*parentSpAccel + pAlpha;
      velDot = hIh.solve(mJointForce - trans(mJointMatrix)*f);
      mChildLink->setAccel(*mParentLink, mJointMatrix*velDot);
    }
  
    /** Compute the articulation step for a given velocity derivative.
     *  Use this for actuators.
     */
    void applyActuatorForce(const VectorN& _velDot)
    {
      // The formulas conform to Roy Featherstones book eqn (7.36), (7.37)
      
      velDot = _velDot;
      
      // Compute the articulated force and inertia.
      // This Since there is no projection step with the joint axis, it is clear
      // that this is just a rigid connection ...
      SpatialInertia I = mChildLink->getInertia();
      Vector6 force = mChildLink->getForce();
      force += I*(mChildLink->getFrame().getHdot() + mJointMatrix*velDot);
      
      // Transform to parent link's coordinates and apply to the parent link
      mParentLink->applyForce(mChildLink->getFrame().forceToParent(force));
      mParentLink->applyInertia(mChildLink->getFrame().inertiaToParent(I));
    }
    
    /** Compute the acceleration step for a given velocity derivative.
     *  Use this for actuators.
     */
    void accelerateDueToVelDot()
    {
      mChildLink->setAccel(*mParentLink, mJointMatrix*velDot);
    }

    const VectorN& getVelDot() const
    { return velDot; }
    
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
    // Stores some values persistent accross velocity/articulation/acceleration
    MatrixFactorsNN hIh;
    Vector6 pAlpha;
    VectorN velDot;
    VectorN mJointForce;

    Vector3 mRelativePosition;

    Matrix6N mJointMatrix;
    
    SharedPtr<MechanicLinkValue> mParentLink;
    SharedPtr<MechanicLinkValue> mChildLink;
    
    SharedPtr<const CartesianJoint> mCartesianJoint;
  };
  
private:
  MechanicLink mParentLink;
  MechanicLink mChildLink;

  Vector3 mPosition;
};

} // namespace OpenFDM

#endif
