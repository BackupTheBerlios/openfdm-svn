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
  newMechanicContext(const MechanicLinkInfo* parentLink,
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
    childLinkValue->setEnvironment(parentLinkValue->getEnvironment());

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

  const Matrix6N& getJointMatrix() const
  { return mJointMatrix; }
  void setJointMatrix(const Matrix6N& jointMatrix)
  { mJointMatrix = jointMatrix; }

  void velocity(const MechanicLinkValue& parentLink,
                MechanicLinkValue& childLink, const Vector3& position,
                const Quaternion& orientation, const Vector6& vel) const
  {
    childLink.setPosAndVel(parentLink, position, orientation, vel);
  }

  /** Compute the articulation step for a given joint force.
   *  Use this for usual joints.
   */
  void articulation(MechanicLinkValue& parentLink,
                    const MechanicLinkValue& childLink,
                    const VectorN& jointForce,
                    MatrixFactorsNN& hIh, Vector6& pAlpha) const
  {
    // The formulas conform to Roy Featherstones book eqn (6.37), (6.38)
    
    // Store the outboard values since we will need them later in velocity
    // derivative computations
    SpatialInertia I = childLink.getInertia();
    
    // Compute the projection to the joint coordinate space
    Matrix6N Ih = I*mJointMatrix;
    hIh = trans(mJointMatrix)*Ih;

    // Note that the momentum of the local mass is already included in the
    // child links force due the the mass model ...
    pAlpha = childLink.getForce() + I*childLink.getFrame().getHdot();
    
    if (hIh.singular()) {
      Log(ArtBody,Error) << "Detected singular mass matrix for "
                         << "CartesianJointFrame \"" << getName()
                         << "\": Fix your model!" << endl;
      return;
    }
    
    // Project away the directions handled with this current joint
    Vector6 force = pAlpha;
    force -= Ih*hIh.solve(trans(mJointMatrix)*pAlpha - jointForce);
    I -= SpatialInertia(Ih*hIh.solve(trans(Ih)));
    
    // Transform to parent link's coordinates and apply to the parent link
    parentLink.applyForce(childLink.getFrame().forceToParent(force));
    parentLink.applyInertia(childLink.getFrame().inertiaToParent(I));
  }

  /** Compute the acceleration step for a given joint force.
   *  Use this for usual joints.
   */
  void acceleration(const MechanicLinkValue& parentLink,
                    MechanicLinkValue& childLink, const VectorN& jointForce,
                    const MatrixFactorsNN& hIh, const Vector6& pAlpha,
                    VectorN& velDot) const
  {
    Vector6 parentSpAccel
      = childLink.getFrame().motionFromParent(parentLink.getFrame().getSpAccel());
    
    Vector6 f = childLink.getInertia()*parentSpAccel + pAlpha;
    velDot = hIh.solve(jointForce - trans(mJointMatrix)*f);
    childLink.setAccel(parentLink, mJointMatrix*velDot);
  }
  
  /** Compute the articulation step for a given velocity derivative.
   *  Use this for actuators.
   */
  void articulation(MechanicLinkValue& parentLink,
                    const MechanicLinkValue& childLink,
                    const VectorN& velDot) const
  {
    // The formulas conform to Roy Featherstones book eqn (7.36), (7.37)

    // Compute the articulated force and inertia.
    // This Since there is no projection step with the joint axis, it is clear
    // that this is just a rigid connection ...
    SpatialInertia I = childLink.getInertia();
    Vector6 force = childLink.getForce();
    force += I*(childLink.getFrame().getHdot() + mJointMatrix*velDot);
    
    // Transform to parent link's coordinates and apply to the parent link
    parentLink.applyForce(childLink.getFrame().forceToParent(force));
    parentLink.applyInertia(childLink.getFrame().inertiaToParent(I));
  }

  /** Compute the acceleration step for a given velocity derivative.
   *  Use this for actuators.
   */
  void acceleration(const MechanicLinkValue& parentLink,
                    MechanicLinkValue& childLink, VectorN& velDot) const
  {
    childLink.setAccel(parentLink, mJointMatrix*velDot);
  }

  virtual void init(const Task&,DiscreteStateValueVector&,
                    ContinousStateValueVector&, const PortValueList&) const
  { }
  virtual void velocity(const MechanicLinkValue& parentLink,
                        MechanicLinkValue& childLink,
                        const ContinousStateValueVector& states,
                        PortValueList& portValues) const = 0;
  virtual void articulation(MechanicLinkValue& parentLink,
                            const MechanicLinkValue& childLink,
                            const ContinousStateValueVector& states,
                            PortValueList& portValues,
                            MatrixFactorsNN& hIh, Vector6& pAlpha) const = 0;
  virtual void acceleration(const MechanicLinkValue& parentLink,
                            MechanicLinkValue& childLink,
                            const ContinousStateValueVector& states,
                            PortValueList& portValues,
                            const MatrixFactorsNN& hIh, const Vector6& pAlpha,
                            VectorN& velDot) const = 0;
  virtual void derivative(const DiscreteStateValueVector&,
                          const ContinousStateValueVector&,
                          const PortValueList& portValues, const VectorN&,
                          ContinousStateValueVector&) const = 0;

private:
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
      mChildLink->setDesignPosition(mCartesianJoint->getPosition());
    }

    bool alloc()
    {
      return allocStates();
    }
    virtual void initVelocities(const /*Init*/Task& task)
    {
      mCartesianJoint->init(task, mDiscreteState, mContinousState, mPortValueList);
      mCartesianJoint->velocity(*mParentLink, *mChildLink,
                                mContinousState, mPortValueList);
    }
    
    virtual void velocities(const Task& task)
    {
      mCartesianJoint->velocity(*mParentLink, *mChildLink,
                                mContinousState, mPortValueList);
    }
    virtual void articulation(const Task& task)
    {
      mCartesianJoint->articulation(*mParentLink, *mChildLink, mContinousState,
                                    mPortValueList, hIh, pAlpha);
    }
    virtual void accelerations(const Task& task)
    {
      mCartesianJoint->acceleration(*mParentLink, *mChildLink, mContinousState,
                                    mPortValueList, hIh, pAlpha, velDot);
    }
    
    virtual void derivative(const Task&)
    {
      mCartesianJoint->derivative(mDiscreteState, mContinousState,
                                  mPortValueList, velDot,
                                  mContinousStateDerivative);
    }
    
    virtual void update(const DiscreteTask&)
    { }
    
  private:
    // Stores some values persistent accross velocity/articulation/acceleration
    MatrixFactorsNN hIh;
    Vector6 pAlpha;
    VectorN velDot;

    SharedPtr<MechanicLinkValue> mParentLink;
    SharedPtr<MechanicLinkValue> mChildLink;
    
    SharedPtr<const CartesianJoint> mCartesianJoint;
  };
  
  MechanicLink mParentLink;
  MechanicLink mChildLink;

  Matrix6N mJointMatrix;
  Vector3 mPosition;
};

} // namespace OpenFDM

#endif
