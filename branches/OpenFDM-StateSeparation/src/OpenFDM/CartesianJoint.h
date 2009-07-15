/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
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
#include "JointContext.h"

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

  virtual JointContext*
  newJointContext(const Environment* environment,
                  MechanicLinkValue* parentLinkValue,
                  MechanicLinkValue* childLinkValue,
                  PortValueList& portValueList) const
  {
    if (!parentLinkValue) {
      Log(Model, Error) << "Parent link is not set while creating context "
                        << "for model \"" << getName() << "\"" << endl;
      return 0;
    }
    if (!childLinkValue) {
      Log(Model, Error) << "Child link is not set while creating context "
                        << "for model \"" << getName() << "\"" << endl;
      return 0;
    }
    SharedPtr<Context> context;
    context = new Context(environment, this,
                          parentLinkValue, childLinkValue, portValueList);
    if (!context->allocStates()) {
      Log(Model, Warning) << "Could not alloc for model \""
                          << getName() << "\"" << endl;
      return 0;
    }
    return context.release();
  }

protected:
  CartesianJoint(const std::string& name) :
    Joint(name),
    mParentLink(new MechanicLink(this, "link0")),
    mChildLink(new MechanicLink(this, "link1"))
  { }
  virtual ~CartesianJoint(void)
  { }

  virtual void init(const Task&, DiscreteStateValueVector&,
                    ContinousStateValueVector&, const PortValueList&) const = 0;
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

  class Context : public JointContext {
  public:
    Context(const Environment* environment,
            const CartesianJoint* cartesianJoint,
            MechanicLinkValue* parentLinkValue,
            MechanicLinkValue* childLinkValue,
            PortValueList& portValueList) :
      JointContext(environment, parentLinkValue, childLinkValue, portValueList),
      mCartesianJoint(cartesianJoint)
    { }
    virtual ~Context() {}
    
    virtual const CartesianJoint& getNode() const
    { return *mCartesianJoint; }
    
    virtual void initDesignPosition()
    {
      mParentLink.setDesignPosition(mCartesianJoint->getPosition());
      mChildLink.setDesignPosition(mCartesianJoint->getPosition());
      mJointMatrix = mCartesianJoint->getJointMatrix();
    }

    virtual void init(const /*Init*/Task& task)
    {
      mCartesianJoint->init(task, mDiscreteState,
                            mContinousState, mPortValueList);
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
    
    void setPosAndVel(const Vector3& position, const Quaternion& orientation,
                      const VectorN& velocity)
    {
      // Set up the local coordinate system of the joint
      Vector3 relPosition = mParentLink.getLinkRelPos() + position;
      mRelativeCoordinateSystem.setPosition(relPosition);
      mRelativeCoordinateSystem.setOrientation(orientation);

      // Propagate the reference coordinate system to the parent.
      mChildLink.setCoordinateSystem(mParentLink.getCoordinateSystem().toReference(mRelativeCoordinateSystem));

      Vector6 relVel = mJointMatrix*velocity;
      mChildLink.setRelativeVelocity(mParentLink, relVel);

      /**
         This is the cross product of the inertial spatial velocity
         vector with the relative spatial velocity vector (motion type
         cross product). Since the inertial velocity is the transformed
         inertial velocity of the parent frame plus the relative
         velocity of the current frame, all the relative velocity
         components cancel out in this expression. What remains is the
         transformed spatial velocity of the parent frame cross the
         relative velocity.
      */
      Vector6 pivel = mParentLink.getLocalVelocityAtLink();
      pivel = mRelativeCoordinateSystem.motionToLocal(pivel);
      mHdot = Vector6(cross(pivel.getAngular(), relVel.getAngular()),
                      cross(pivel.getAngular(), relVel.getLinear()) + 
                      cross(pivel.getLinear(), relVel.getAngular()));
    }

    /** This is the derivative of the joint matrix times the joint velocity
     */
    const Vector6& getHdot() const
    { return mHdot; }
    
    /** Compute the articulation step for a given joint force.
     *  Use this for usual joints.
     */
    void applyJointForce(const VectorN& jointForce)
    {
      // The formulas conform to Roy Featherstones book eqn (6.37), (6.38)

      mJointForce = jointForce;
    
      // Store the outboard values since we will need them later in velocity
      // derivative computations
      SpatialInertia I = mChildLink.getInertia();
      
      // Compute the projection to the joint coordinate space
      Matrix6N Ih = I*mJointMatrix;
      hIh = trans(mJointMatrix)*Ih;

      // Note that the momentum of the local mass is already included in the
      // child links force due the the mass model ...
      pAlpha = mChildLink.getForce() + I*getHdot();
      
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
      force = mRelativeCoordinateSystem.forceToReference(force);
      I = mRelativeCoordinateSystem.inertiaToReference(I);
      mParentLink.addForceAtLink(force);
      mParentLink.addInertiaAtLink(I);
    }

    /** Compute the acceleration step for a given joint force.
     *  Use this for usual joints.
     */
    void accelerateDueToForce()
    {
      if (hIh.singular())
        return;

      Vector6 parentSpAccel = mParentLink.getLocalAccelerationAtLink();
      parentSpAccel = mRelativeCoordinateSystem.motionToLocal(parentSpAccel);

      Vector6 f = mChildLink.getInertia()*parentSpAccel + pAlpha;
      mVelDot = hIh.solve(mJointForce - trans(mJointMatrix)*f);

      Vector6 spAccel = parentSpAccel + getHdot() + mJointMatrix*mVelDot;
      mChildLink.setLocalAcceleration(spAccel);
    }
  
    /** Compute the articulation step for a given velocity derivative.
     *  Use this for actuators.
     */
    void applyActuatorForce(const VectorN& velDot)
    {
      // The formulas conform to Roy Featherstones book eqn (7.36), (7.37)
      
      mVelDot = velDot;
      
      // Compute the articulated force and inertia.
      // This Since there is no projection step with the joint axis, it is clear
      // that this is just a rigid connection ...
      SpatialInertia I = mChildLink.getInertia();
      Vector6 force = mChildLink.getForce();
      force += I*(getHdot() + mJointMatrix*mVelDot);
      
      // Transform to parent link's coordinates and apply to the parent link
      force = mRelativeCoordinateSystem.forceToReference(force);
      I = mRelativeCoordinateSystem.inertiaToReference(I);
      mParentLink.addForceAtLink(force);
      mParentLink.addInertiaAtLink(I);
    }
    
    /** Compute the acceleration step for a given velocity derivative.
     *  Use this for actuators.
     */
    void accelerateDueToVelDot()
    {
      Vector6 parentSpAccel = mParentLink.getLocalAccelerationAtLink();
      parentSpAccel = mRelativeCoordinateSystem.motionToLocal(parentSpAccel);

      Vector6 spAccel = parentSpAccel + getHdot() + mJointMatrix*mVelDot;
      mChildLink.setLocalAcceleration(spAccel);
    }

    const VectorN& getVelDot() const
    { return mVelDot; }

  private:
    // Stores some values persistent accross velocity/articulation/acceleration
    MatrixFactorsNN hIh;
    Vector6 mHdot;
    Vector6 pAlpha;
    VectorN mVelDot;
    VectorN mJointForce;

    CoordinateSystem mRelativeCoordinateSystem;

    Matrix6N mJointMatrix;
    
    SharedPtr<const CartesianJoint> mCartesianJoint;
  };
  
private:
  SharedPtr<MechanicLink> mParentLink;
  SharedPtr<MechanicLink> mChildLink;
};

} // namespace OpenFDM

#endif
