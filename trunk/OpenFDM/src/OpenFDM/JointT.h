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
                         const Vector6& jointForce,
                         const Vector6& frameDotQ,
                         const Matrix6N& jointAxis)
  {
    Matrix6N Ih = artI*jointAxis;
    hIh = trans(jointAxis)*Ih;
    
    if (hIh.singular())
      return false;

    artF += artI*frameDotQ;
    Vector6 pAlpha = artF;
    
    mForcePAlpha = jointForce - pAlpha;
    artF += Ih*hIh.solve(trans(jointAxis)*mForcePAlpha);
    artI -= SpatialInertia(Ih*hIh.solve(trans(Ih)));
    return true;
  }
  
  void computeRelAccel(const SpatialInertia& outBoardArtInertia,
                       const Vector6& parentSpAccel,
                       const Matrix6N& jointAxis,
                       VectorN& jointAccel) const
  {
    if (hIh.singular()) {
      jointAccel.clear();
    } else {
      Vector6 tmp = mForcePAlpha - outBoardArtInertia*parentSpAccel;
      jointAccel = hIh.solve(trans(jointAxis)*tmp);
    }
  }

private:
  Vector6 mForcePAlpha;
  MatrixFactorsNN hIh;
};

} // namespace OpenFDM

#endif
