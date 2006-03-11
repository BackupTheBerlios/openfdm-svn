/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_CartesianActuatorFrame_H
#define OpenFDM_CartesianActuatorFrame_H

#include "Assert.h"
#include "Vector.h"
#include "Matrix.h"
#include "Inertia.h"
#include "Frame.h"

namespace OpenFDM {

template<unsigned n>
class CartesianActuatorFrame :
  public Frame {
public:
  typedef LinAlg::Vector<real_type,n> VectorN;
  typedef LinAlg::Matrix<real_type,6,n> Matrix6N;
  typedef LinAlg::Matrix<real_type,n,n> MatrixNN;
  typedef LinAlg::MatrixFactors<real_type,n,n,LinAlg::LUTag> MatrixFactorsNN;

  CartesianActuatorFrame(const std::string& name) :
    Frame(name),
//     mJointMatrix(Matrix6N::zeros()), /// ??? ... see LinAlg checkout ...
    mOutboardInertia(SpatialInertia::zeros()),
    mJointAccel(VectorN::zeros()),
    mArticulationDirty(true),
    mJointVelDotDirty(true),
    mSpVelDotDirty(true),
    mJointVelDot(VectorN::zeros()),
    mRelVelDot(Vector6::zeros())
  { }
  virtual ~CartesianActuatorFrame(void)
  { }

  /// The interface routine for the Frame,
  /// returns the relative velocity derivative of this frame
  virtual const Vector6& getRelVelDot(void) const
  {
    if (mSpVelDotDirty) {
      mRelVelDot = mJointMatrix*getJointVelDot();
      mSpVelDotDirty = false;
      // Note that we do not need to mark the accelerations dirty since
      // we can only get here if something else the accelerations will depend
      // on anyway is set dirty before
      // setAccelDirty();
    }
    return mRelVelDot;
  }

  /// Returns the derivative of the joint velocity
  const VectorN& getJointVelDot() const
  {
    OpenFDMAssert(!mArticulationDirty);
    if (mJointVelDotDirty) {
      if (hIh.singular()) {
        Log(ArtBody,Error) << "Detected singular mass matrix for "
                           << "CartesianActuatorFrame \"" << getName()
                           << "\": Fix your model!" << endl;
        mJointVelDot.clear();
      } else {
        Vector6 tmp = mOutboardInertia*getParentSpAccel();
        mJointVelDot = mJointAccel - hIh.solve(trans(mJointMatrix)*tmp);
      }
      mJointVelDotDirty = false;
    }
    return mJointVelDot;
  }

  /// Compute the articulated force and inertia past inboard to that joint
  bool jointArticulation(SpatialInertia& artI, Vector6& artF,
                         const Vector6& outF, const SpatialInertia& outI,
                         const VectorN& jointAccel)
  {
    // Store the outboard values since we will need them later in velocity
    // derivative computations
    mOutboardInertia = outI;
    mJointAccel = jointAccel;
    // Make sure we have the correct internal state
    mJointVelDotDirty = true;
    mArticulationDirty = false;

    // Compute the projection to the joint coodinate space
    Matrix6N Ih = outI*mJointMatrix;
    hIh = trans(mJointMatrix)*Ih;

    artF = outF + mOutboardInertia*getHdot();
    artI = outI;

    if (hIh.singular()) {
      Log(ArtBody,Error) << "Detected singular mass matrix for "
                         << "CartesianActuatorFrame \"" << getName()
                         << "\": Fix your model!" << endl;
      return false;
    }
    
    // Project away the directions handled with this current joint
    artF += Ih*mJointAccel;
    artI -= SpatialInertia(Ih*hIh.solve(trans(Ih)));

    return true;
  }

protected:
  const Matrix6N& getJointMatrix(void) const
  { return mJointMatrix; }

  void setJointMatrix(const Matrix6N& jointAxis)
  { mJointMatrix = jointAxis; setDirty(); }

  void setPosition(const Vector3& pos)
  { Frame::setPosition(pos); setDirty(); }
  void setOrientation(const Quaternion& orientation)
  { Frame::setOrientation(orientation); setDirty(); }
  void setRelVel(const Vector6& vel)
  { Frame::setRelVel(vel); setDirty(); }
  void setLinearRelVel(const Vector3& vel)
  { Frame::setLinearRelVel(vel); setDirty(); }
  void setAngularRelVel(const Vector3& vel)
  { Frame::setAngularRelVel(vel); setDirty(); }
  void setDirty(void) const
  {
    mArticulationDirty = true;
    mJointVelDotDirty = true;
    mSpVelDotDirty = true;
    setAccelDirty();
  }

private:
  /// The cartesian joint map matrix, that is for the simple one dimensional
  /// case just a spatial vector.
  Matrix6N mJointMatrix;

  /// The articulated intertia of the outboard frame, 
  SpatialInertia mOutboardInertia;
  /// The joint internal force in joint generalized coordinates
  VectorN mJointAccel;
  /// The decomposition of the inertia matrix projected to joint coordinates
  MatrixFactorsNN hIh;
  /// This is true if the state has changed but the articulated intertia and
  /// forces are not yet updated
  mutable bool mArticulationDirty:1;
  /// This is true if the joint velocity derivatives are not yet computed
  mutable bool mJointVelDotDirty:1;
  /// This is true if the spatial velocity derivative is not yet computed
  mutable bool mSpVelDotDirty:1;
  /// The derivative of the joint velocity
  mutable VectorN mJointVelDot;
  /// The derivative of the frame velocity
  mutable Vector6 mRelVelDot;
};

} // namespace OpenFDM

#endif
