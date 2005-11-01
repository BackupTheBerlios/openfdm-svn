/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_JointT_H
#define OpenFDM_JointT_H

#include "Assert.h"
#include "Vector.h"
#include "Matrix.h"
#include "Inertia.h"

namespace OpenFDM {

template<unsigned n>
class JointT {
  // FIXME: move that as template member into RigidBody ...
  // or something like that ...
protected:
  typedef LinAlg::Vector<real_type,n> VectorN;
  typedef LinAlg::Matrix<real_type,6,n> Matrix6N;
  typedef LinAlg::Matrix<real_type,n,n> MatrixNN;
  typedef LinAlg::MatrixFactors<real_type,n,n,LinAlg::LUTag> MatrixFactorsNN;

  bool jointArticulation(SpatialInertia& artI,
                         Vector6& artF,
                         const Vector6& pAlpha,
                         const Vector6& jointForce,
                         const Matrix6N& jointAxis)
  {
    Log(ArtBody, Debug1) << artI << endl;

    mJointForce = jointForce;

    Matrix6N Ih = artI*jointAxis;
    hIh = trans(jointAxis)*Ih;

    if (hIh.singular())
      return false;

    artF = pAlpha;
    
    Vector6 mForcePAlpha = pAlpha - jointForce;

    Log(ArtBody, Debug1) << trans(jointAxis)*Ih
                         << endl
                         << trans(jointForce)
                         << endl
                         << trans(pAlpha)
                         << endl
                         << trans(Ih*hIh.solve(trans(jointAxis)*mForcePAlpha))
                         << endl
                         << SpatialInertia(Ih*hIh.solve(trans(Ih)))
                         << endl;


    artF -= Ih*hIh.solve(trans(jointAxis)*mForcePAlpha);
    artI -= SpatialInertia(Ih*hIh.solve(trans(Ih)));

    return true;
  }
  
  void computeRelAccel(const SpatialInertia& outBoardArtInertia,
                       const Vector6& parentSpAccel,
                       const Vector6& pAlpha,
                       const Matrix6N& jointAxis,
                       VectorN& jointAccel) const
  {
    if (hIh.singular()) {
      jointAccel.clear();
    } else {
      Vector6 tmp = mJointForce - outBoardArtInertia*parentSpAccel - pAlpha;
      jointAccel = hIh.solve(trans(jointAxis)*tmp);
    }
  }

private:
  Vector6 mJointForce;
  MatrixFactorsNN hIh;
};

} // namespace OpenFDM

#endif
