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
  typedef LinAlg::Vector<real_type,n> VectorN;
  typedef LinAlg::Matrix<real_type,6,n> Matrix6N;
  typedef LinAlg::Matrix<real_type,n,n> MatrixNN;
  typedef LinAlg::MatrixFactors<real_type,n,n,LinAlg::LUTag> MatrixFactorsNN;

  CartesianJointFrame(const std::string& name) :
    Frame(name)
  { }
  virtual ~CartesianJointFrame(void)
  { }

protected:
  const Matrix6N& getJointMatrix(void) const
  { return mJointMatrix; }

  void setJointMatrix(const Matrix6N& jointAxis)
  { mJointMatrix = jointAxis; }

public: /// FIXME
  bool jointArticulation(SpatialInertia& artI,
                         Vector6& artF,
                         const Vector6& outF,
                         const SpatialInertia& outI,
                         const VectorN& jointForce)
  {
    Log(ArtBody, Debug1) << artI << endl;

    mOutboardInertia = outI;
    mOutboardForce = outF;


    mPAlpha = outF + mOutboardInertia*getHdot();



    mJointForce = jointForce;

    Matrix6N Ih = outI*mJointMatrix;
    hIh = trans(mJointMatrix)*Ih;

    if (hIh.singular())
      return false;

    artF = mPAlpha;
    
    artF -= Ih*hIh.solve(trans(mJointMatrix)*mPAlpha - jointForce);
    artI = outI;
    artI -= SpatialInertia(Ih*hIh.solve(trans(Ih)));

    return true;
  }
  
  void computeRelVelDot(VectorN& jointAccel) const
  {
    if (hIh.singular()) {
      jointAccel.clear();
    } else {
      Vector6 tmp = mOutboardInertia*getParentSpAccel() + mPAlpha;
      jointAccel = hIh.solve(mJointForce - trans(mJointMatrix)*tmp);
    }
  }

private:
  SpatialInertia mOutboardInertia;
  Vector6 mOutboardForce;
  Vector6 mPAlpha;
  Matrix6N mJointMatrix;
  VectorN mJointForce;
  MatrixFactorsNN hIh;
};

} // namespace OpenFDM

#endif
