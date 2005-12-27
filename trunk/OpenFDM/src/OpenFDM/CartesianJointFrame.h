/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_CartesianJointFrame_H
#define OpenFDM_CartesianJointFrame_H

#include "Assert.h"
#include "Vector.h"
#include "Matrix.h"
#include "Inertia.h"
#include "Frame.h"

namespace OpenFDM {

template<unsigned n>
class CartesianJointFrame :
  public Frame {
public:
  CartesianJointFrame(const std::string& name) :
    Frame(name)
  { }
  virtual ~CartesianJointFrame(void)
  { }


// protected:
  typedef LinAlg::Vector<real_type,n> VectorN;
  typedef LinAlg::Matrix<real_type,6,n> Matrix6N;
  typedef LinAlg::Matrix<real_type,n,n> MatrixNN;
  typedef LinAlg::MatrixFactors<real_type,n,n,LinAlg::LUTag> MatrixFactorsNN;

  bool jointArticulation(SpatialInertia& artI,
                         Vector6& artF,
                         const Vector6& outF,
                         const SpatialInertia& outI,
                         const VectorN& jointForce,
                         const Matrix6N& jointAxis)
  {
    Log(ArtBody, Debug1) << artI << endl;

    mOutboardInertia = outI;
    mOutboardForce = outF;


    Vector6 pAlpha = outF + mOutboardInertia*getHdot();



    mJointForce = jointForce;

    Matrix6N Ih = outI*jointAxis;
    hIh = trans(jointAxis)*Ih;

    if (hIh.singular())
      return false;

    artF = pAlpha;
    
    artF -= Ih*hIh.solve(trans(jointAxis)*pAlpha - jointForce);
    artI = outI;
    artI -= SpatialInertia(Ih*hIh.solve(trans(Ih)));

    return true;
  }
  
  void computeRelAccel(const Matrix6N& jointAxis,
                       VectorN& jointAccel) const
  {
    if (hIh.singular()) {
      jointAccel.clear();
    } else {
      Vector6 pAlpha = mOutboardForce + mOutboardInertia*getHdot();
      Vector6 tmp = - mOutboardInertia*getParentSpAccel() - pAlpha;
      jointAccel = hIh.solve(trans(jointAxis)*tmp + mJointForce);
    }
  }

private:
  SpatialInertia mOutboardInertia;
  Vector6 mOutboardForce;
  VectorN mJointForce;
  MatrixFactorsNN hIh;
};

} // namespace OpenFDM

#endif
