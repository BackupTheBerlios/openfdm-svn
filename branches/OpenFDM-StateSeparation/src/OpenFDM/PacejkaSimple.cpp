/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "PacejkaSimple.h"

#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(PacejkaSimple, PacejkaTire)
  DEF_OPENFDM_PROPERTY(Real, SpringConstant, Serialized)
  DEF_OPENFDM_PROPERTY(Real, DampingConstant, Serialized)
  DEF_OPENFDM_PROPERTY(Real, Bx, Serialized)
  DEF_OPENFDM_PROPERTY(Real, Cx, Serialized)
  DEF_OPENFDM_PROPERTY(Real, Dx, Serialized)
  DEF_OPENFDM_PROPERTY(Real, Ex, Serialized)
  DEF_OPENFDM_PROPERTY(Real, By, Serialized)
  DEF_OPENFDM_PROPERTY(Real, Cy, Serialized)
  DEF_OPENFDM_PROPERTY(Real, Dy, Serialized)
  DEF_OPENFDM_PROPERTY(Real, Ey, Serialized)
  END_OPENFDM_OBJECT_DEF

PacejkaSimple::PacejkaSimple(const std::string& name) :
  PacejkaTire(name),
  mSpringConstant(0),
  mDampingConstant(0),
  mBx(1), mCx(0.01), mDx(1), mEx(0),
  mBy(10), mCy(5), mDy(1), mEy(0)
{
}

PacejkaSimple::~PacejkaSimple(void)
{
}

Vector6
PacejkaSimple::getForce(const real_type& rho, const real_type& rhoDot,
                        const real_type& alpha, const real_type& kappa,
                        const real_type& gamma, const real_type& phi) const
{
  // The normal force
  real_type Fz = rho*mSpringConstant + mDampingConstant*rhoDot;
  // The normal force cannot get negative here.
  Fz = max(real_type(0), Fz);

  //
  // Longitudinal force (Pure longitudinal slip)
  //
  real_type Bxk = mBx*100*kappa;
  real_type Fx = Fz*mDx*sin(mCx*atan(Bxk - mEx*(Bxk - atan(Bxk))));

  //
  // Lateral force (Pure side slip)
  //
  real_type Bya = mBy*rad2deg*alpha;
  real_type Fy = Fz*mDy*sin(mCy*atan(Bya - mEy*(Bya - atan(Bya))));

  return Vector6(Vector3(0, 0, 0), Vector3(Fx, Fy, Fz));
}

void
PacejkaSimple::setSpringConstant(const real_type& springConstant)
{
  mSpringConstant = springConstant;
}

const real_type&
PacejkaSimple::getSpringConstant(void) const
{
  return mSpringConstant;
}

void
PacejkaSimple::setDampingConstant(const real_type& dampingConstant)
{
  mDampingConstant = dampingConstant;
}

const real_type&
PacejkaSimple::getDampingConstant(void) const
{
  return mDampingConstant;
}

} // namespace OpenFDM
