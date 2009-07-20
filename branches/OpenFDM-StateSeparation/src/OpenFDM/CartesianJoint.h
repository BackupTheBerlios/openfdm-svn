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

      // Propagate the reference coordinate system to the parent.
      mChildLink.setCoordinateSystem(mParentLink.getCoordinateSystem().getRelative(relPosition, orientation));

      const CoordinateSystem& cs = mChildLink.getCoordinateSystem();
      const CoordinateSystem& ps = mParentLink.getCoordinateSystem();

      mJointMatrix = cs.rotToReference(mCartesianJoint->getJointMatrix());

      Vector3 positionDifference = cs.getPosition() - ps.getPosition();

      Vector6 parentSpVel = mParentLink.getVelocity();
      parentSpVel = motionTo(positionDifference, parentSpVel);

      Vector6 relVel = mJointMatrix*velocity;
      mChildLink.setVelocity(parentSpVel + relVel);

      Vector6 parentInVel = mParentLink.getInertialVelocity();
      parentInVel = motionTo(positionDifference, parentInVel);
      mChildLink.setInertialVelocity(parentInVel + relVel);

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
      mHdot = Vector6(cross(parentInVel.getAngular(), relVel.getAngular()),
                      cross(parentInVel.getAngular(), relVel.getLinear()) + 
                      cross(parentInVel.getLinear(), relVel.getAngular()));


      mChildLink.setForce(Vector6::zeros());
      mChildLink.setInertia(SpatialInertia::zeros());
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

      const CoordinateSystem& cs = mChildLink.getCoordinateSystem();

      // Compute the projection of the inertia matrix to the
      // joint coordinate space
      SpatialInertia I = mChildLink.getInertia();
      Matrix6N Ih = I*mJointMatrix;
      hIh = trans(mJointMatrix)*Ih;

      if (hIh.singular()) {
        Log(ArtBody,Error) << "Detected singular mass matrix for "
                           << "CartesianJoint \"" << mCartesianJoint->getName()
                           << "\": Fix your model!" << endl;
        return;
      }

      // Note that the momentum of the local mass is already included in the
      // child links force due the the mass model ...
      Vector6 force = mChildLink.getForce() + I*getHdot();

      // Project away those axis handled with this current joint
      force -= Ih*hIh.solve(trans(mJointMatrix)*force - jointForce);
      I -= SpatialInertia(Ih*hIh.solve(trans(Ih)));

      // Contribute the remaining force and inertiy to the parent link
      mParentLink.addForce(cs.getPosition(), force);
      mParentLink.addInertia(cs.getPosition(), I);
    }

    /** Compute the acceleration step for a given joint force.
     *  Use this for usual joints.
     */
    void accelerateDueToForce()
    {
      if (hIh.singular())
        return;

      const CoordinateSystem& cs = mChildLink.getCoordinateSystem();
      const CoordinateSystem& ps = mParentLink.getCoordinateSystem();

      Vector3 positionDifference = cs.getPosition() - ps.getPosition();

      Vector6 parentSpAccel = mParentLink.getInertialAcceleration();
      parentSpAccel = motionTo(positionDifference, parentSpAccel);

      Vector6 f = mChildLink.getForce();
      f += mChildLink.getInertia()*(parentSpAccel + getHdot());

      mVelDot = hIh.solve(mJointForce - trans(mJointMatrix)*f);

      Vector6 spAccel = parentSpAccel + getHdot() + mJointMatrix*mVelDot;
      mChildLink.setInertialAcceleration(spAccel);
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

      const CoordinateSystem& cs = mChildLink.getCoordinateSystem();

      const SpatialInertia& I = mChildLink.getInertia();
      Vector6 force = mChildLink.getForce();
      force += I*(getHdot() + mJointMatrix*mVelDot);
      
      mParentLink.addForce(cs.getPosition(), force);
      mParentLink.addInertia(cs.getPosition(), I);
    }
    
    /** Compute the acceleration step for a given velocity derivative.
     *  Use this for actuators.
     */
    void accelerateDueToVelDot()
    {
      const CoordinateSystem& cs = mChildLink.getCoordinateSystem();
      const CoordinateSystem& ps = mParentLink.getCoordinateSystem();
      Vector3 positionDifference = cs.getPosition() - ps.getPosition();

      Vector6 parentSpAccel = mParentLink.getInertialAcceleration();
      parentSpAccel = motionTo(positionDifference, parentSpAccel);

      Vector6 spAccel = parentSpAccel + getHdot() + mJointMatrix*mVelDot;
      mChildLink.setInertialAcceleration(spAccel);
    }

    const VectorN& getVelDot() const
    { return mVelDot; }

  private:
    // Stores some values persistent accross velocity/articulation/acceleration
    MatrixFactorsNN hIh;
    Vector6 mHdot;
    VectorN mVelDot;
    VectorN mJointForce;

    Matrix6N mJointMatrix;
    
    SharedPtr<const CartesianJoint> mCartesianJoint;
  };
  
private:
  SharedPtr<MechanicLink> mParentLink;
  SharedPtr<MechanicLink> mChildLink;
};

} // namespace OpenFDM

#endif
