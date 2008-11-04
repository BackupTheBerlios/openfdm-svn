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
public:
  typedef LinAlg::Vector<real_type,n> VectorN;
  typedef LinAlg::Matrix<real_type,6,n> Matrix6N;
  typedef LinAlg::Matrix<real_type,n,n> MatrixNN;
  typedef LinAlg::MatrixFactors<real_type,n,n,LinAlg::LUTag> MatrixFactorsNN;

protected:
  CartesianJoint(const std::string& name, const Matrix6N& jointMatrix) :
    Joint(name),
    mJointMatrix(jointMatrix)
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
                    Matrix& hIh) const
  {
    // The formulas conform to Roy Featherstones book eqn (6.37), (6.38)
    
    // Store the outboard values since we will need them later in velocity
    // derivative computations
    SpatialInertia I = childLink.getInertia();
    
    // Compute the projection to the joint coordinate space
    Matrix6N Ih = I*mJointMatrix;
    hIh = trans(mJointMatrix)*Ih;
    MatrixFactorsNN hIhFac = MatrixNN(hIh);
    
    // Note that the momentum of the local mass is already included in the
    // child links force due the the mass model ...
    Vector6 pAlpha = childLink.getForce() + I*childLink.getFrame().getHdot();
    Vector6 force = pAlpha;
    
    if (hIhFac.singular()) {
      Log(ArtBody,Error) << "Detected singular mass matrix for "
                         << "CartesianJointFrame \"" << getName()
                         << "\": Fix your model!" << endl;
      return;
    }
    
    // Project away the directions handled with this current joint
    force -= Ih*hIhFac.solve(trans(mJointMatrix)*pAlpha - jointForce);
    I -= SpatialInertia(Ih*hIhFac.solve(trans(Ih)));
    
    // Transform to parent link's coordinates and apply to the parent link
    parentLink.applyForce(childLink.getFrame().forceToParent(force));
    parentLink.applyInertia(childLink.getFrame().inertiaToParent(I));
  }

  /** Compute the acceleration step for a given joint force.
   *  Use this for usual joints.
   */
  void acceleration(const MechanicLinkValue& parentLink,
                    MechanicLinkValue& childLink, const VectorN& jointForce,
                    const Matrix& hIh, Vector& velDot) const
  {
    Vector6 parentSpAccel
      = childLink.getFrame().motionFromParent(parentLink.getFrame().getSpAccel());
    
    Vector6 f = childLink.getForce();
    f += childLink.getInertia()*(parentSpAccel + childLink.getFrame().getHdot());
    MatrixFactorsNN hIhFac = MatrixNN(hIh);
    velDot = hIhFac.solve(jointForce - trans(mJointMatrix)*f);
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

private:
  Matrix6N mJointMatrix;
};

} // namespace OpenFDM

#endif
