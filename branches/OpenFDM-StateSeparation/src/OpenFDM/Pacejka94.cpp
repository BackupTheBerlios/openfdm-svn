/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "Pacejka94.h"

#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Pacejka94, PacejkaTire)
  DEF_OPENFDM_PROPERTY(Real, SpringConstant, Serialized)
  DEF_OPENFDM_PROPERTY(Real, DampingConstant, Serialized)
  DEF_OPENFDM_PROPERTY(Real, FzMin, Serialized)
  DEF_OPENFDM_PROPERTY(Real, A0, Serialized)
  DEF_OPENFDM_PROPERTY(Real, A1, Serialized)
  DEF_OPENFDM_PROPERTY(Real, A2, Serialized)
  DEF_OPENFDM_PROPERTY(Real, A15, Serialized)
  DEF_OPENFDM_PROPERTY(Real, A3, Serialized)
  DEF_OPENFDM_PROPERTY(Real, A4, Serialized)
  DEF_OPENFDM_PROPERTY(Real, A5, Serialized)
  DEF_OPENFDM_PROPERTY(Real, A6, Serialized)
  DEF_OPENFDM_PROPERTY(Real, A7, Serialized)
  DEF_OPENFDM_PROPERTY(Real, A16, Serialized)
  DEF_OPENFDM_PROPERTY(Real, A17, Serialized)
  DEF_OPENFDM_PROPERTY(Real, A8, Serialized)
  DEF_OPENFDM_PROPERTY(Real, A9, Serialized)
  DEF_OPENFDM_PROPERTY(Real, A10, Serialized)
  DEF_OPENFDM_PROPERTY(Real, A11, Serialized)
  DEF_OPENFDM_PROPERTY(Real, A12, Serialized)
  DEF_OPENFDM_PROPERTY(Real, A13, Serialized)
  DEF_OPENFDM_PROPERTY(Real, A14, Serialized)
  DEF_OPENFDM_PROPERTY(Real, DLAT, Serialized)
  DEF_OPENFDM_PROPERTY(Real, BCDLAT, Serialized)
  DEF_OPENFDM_PROPERTY(Real, B0, Serialized)
  DEF_OPENFDM_PROPERTY(Real, B1, Serialized)
  DEF_OPENFDM_PROPERTY(Real, B2, Serialized)
  DEF_OPENFDM_PROPERTY(Real, B3, Serialized)
  DEF_OPENFDM_PROPERTY(Real, B4, Serialized)
  DEF_OPENFDM_PROPERTY(Real, B5, Serialized)
  DEF_OPENFDM_PROPERTY(Real, B6, Serialized)
  DEF_OPENFDM_PROPERTY(Real, B7, Serialized)
  DEF_OPENFDM_PROPERTY(Real, B8, Serialized)
  DEF_OPENFDM_PROPERTY(Real, B13, Serialized)
  DEF_OPENFDM_PROPERTY(Real, B9, Serialized)
  DEF_OPENFDM_PROPERTY(Real, B10, Serialized)
  DEF_OPENFDM_PROPERTY(Real, B11, Serialized)
  DEF_OPENFDM_PROPERTY(Real, B12, Serialized)
  DEF_OPENFDM_PROPERTY(Real, DLON, Serialized)
  DEF_OPENFDM_PROPERTY(Real, BCDLON, Serialized)
  DEF_OPENFDM_PROPERTY(Real, C0, Serialized)
  DEF_OPENFDM_PROPERTY(Real, C1, Serialized)
  DEF_OPENFDM_PROPERTY(Real, C2, Serialized)
  DEF_OPENFDM_PROPERTY(Real, C18, Serialized)
  DEF_OPENFDM_PROPERTY(Real, C3, Serialized)
  DEF_OPENFDM_PROPERTY(Real, C4, Serialized)
  DEF_OPENFDM_PROPERTY(Real, C5, Serialized)
  DEF_OPENFDM_PROPERTY(Real, C6, Serialized)
  DEF_OPENFDM_PROPERTY(Real, C7, Serialized)
  DEF_OPENFDM_PROPERTY(Real, C8, Serialized)
  DEF_OPENFDM_PROPERTY(Real, C9, Serialized)
  DEF_OPENFDM_PROPERTY(Real, C10, Serialized)
  DEF_OPENFDM_PROPERTY(Real, C19, Serialized)
  DEF_OPENFDM_PROPERTY(Real, C20, Serialized)
  DEF_OPENFDM_PROPERTY(Real, C11, Serialized)
  DEF_OPENFDM_PROPERTY(Real, C12, Serialized)
  DEF_OPENFDM_PROPERTY(Real, C13, Serialized)
  DEF_OPENFDM_PROPERTY(Real, C14, Serialized)
  DEF_OPENFDM_PROPERTY(Real, C15, Serialized)
  DEF_OPENFDM_PROPERTY(Real, C16, Serialized)
  DEF_OPENFDM_PROPERTY(Real, C17, Serialized)
  END_OPENFDM_OBJECT_DEF

Pacejka94::Pacejka94(const std::string& name) :
  PacejkaTire(name),
  mSpringConstant(0),
  mDampingConstant(0),

  mFzMin(0),

  mA0(2),
  mA1(0), mA2(1000), mA15(0),
  mA3(1000), mA4(10), mA5(0),
  mA6(0), mA7(0.5), mA16(0), mA17(0),
  mA8(0), mA9(0), mA10(0),
  mA11(0), mA12(0), mA13(0), mA14(0),
  mDLAT(1), mBCDLAT(1),

  mB0(2),
  mB1(0), mB2(1000),
  mB3(0), mB4(10), mB5(0),
  mB6(0), mB7(0), mB8(0.5), mB13(0),
  mB9(0), mB10(0),
  mB11(0), mB12(0),
  mDLON(1), mBCDLON(1),

  mC0(1),
  mC1(0), mC2(1), mC18(0),
  mC3(0), mC4(0), mC5(0), mC6(0),
  mC7(0), mC8(0), mC9(0), mC10(0), mC19(0), mC20(0),
  mC11(0), mC12(0), mC13(0),
  mC14(0), mC15(0), mC16(0), mC17(0)
{
}

Pacejka94::~Pacejka94(void)
{
}

Vector6
Pacejka94::getForce(const real_type& rho, const real_type& rhoDot,
                    const real_type& alpha, const real_type& kappa,
                    const real_type& gamma, const real_type& phi) const
{
  // Pacejka suggests this correction to avoid singularities at zero speed
  const real_type e_v = 1e-2;

  // The normal force
  real_type Fz = rho*mSpringConstant + mDampingConstant*rhoDot;
  // The normal force cannot get negative here.
  Fz = max(real_type(0), Fz);

  // Pacejka's equations tend to provide infinitesimal stiff tires at low
  // normal forces. Limit the input to the magic formula to a minimal
  // normal load and rescale the output force according to the original
  // load.
  real_type FzRatio = 1;
  if (Fz < mFzMin) {
    FzRatio = Fz/mFzMin;
    Fz = mFzMin;
  }

  // The normal force is mean to be in kN
  Fz *= 1e-3;

  //
  // Longitudinal force (Pure longitudinal slip)
  //
  // Shape factor
  real_type Cx = mB0;
  // Peak factor
  real_type Dx = (mB1*Fz + mB2)*Fz*mDLON;
  // BCD
  real_type BCDx = (mB3*Fz + mB4)*Fz*exp(-mB5*Fz)*mBCDLON;
  // Stiffness factor
  real_type Bx = BCDx/(Cx*Dx + e_v);
  // Horizonal shift
  real_type SHx = (mB9*Fz + mB10);
  // Vertical shift
  real_type SVx = (mB11*Fz + mB12);
  // Shifted longitudinal slip
  // FIXME is this really in % ??? Also the other old model!!!
  real_type kappax = 100*kappa + SHx;
  // Curvature factor
  real_type Ex = ((mB6*Fz + mB7)*Fz + mB8)*(1 - mB13*copysign(1, kappax));
  // See P175 Note on Fig 4.10: Clamp E <= 1 to avoid unrealistic behaviour
  Ex = min(Ex, real_type(1));
  // The resulting longitudinal force
  real_type Bxk = Bx*kappax;
  real_type Fx = Dx*sin(Cx*atan(Bxk - Ex*(Bxk - atan(Bxk)))) + SVx;

  //
  // Lateral force (Pure side slip)
  //
  // Shape factor
  real_type Cy = mA0;
  // Peak factor
  real_type Dy = (mA1*Fz + mA2)*(1 - mA15*sqr(gamma))*Fz*mDLAT;
  // BCD
  real_type BCDy = mA3*sin(atan(Fz/mA4)*2)*(1 - mA5*fabs(gamma))*mBCDLAT;
  // Stiffness factor
  real_type By = BCDy/(Cy*Dy + e_v);
  // Horizonal shift
  real_type SHy = mA8*Fz + mA9 + mA10*gamma;
  // Vertical shift
  real_type SVy = mA11*Fz + mA12 + (mA13*Fz + mA14)*Fz*gamma;
  // Shifted lateral slip
  real_type alphay = rad2deg*alpha + SHy;
  // Curvature factor
  real_type Ey = (mA6*Fz + mA7)*(1 - (mA16*gamma + mA17)*copysign(1, alphay));
  // See P175 Note on Fig 4.10: Clamp E <= 1 to avoid unrealistic behaviour
  Ey = min(Ey, real_type(1));
  // The resulting lateral force
  real_type Bya = By*alphay;
  real_type Fy = Dy*sin(Cy*atan(Bya - Ey*(Bya - atan(Bya)))) + SVy;

  //
  // Self aligning torque
  //
  // Shape factor
  real_type Cz = mC0;
  // Peak factor
  real_type Dz = (mC1*Fz + mC2)*Fz*(1 - mC18*sqr(gamma));
  // BCD
  real_type BCDz = (mC3*Fz + mC4)*Fz*(1 - mC6*fabs(gamma))*exp(-mC5*Fz);
  // Stiffness factor
  real_type Bz = BCDz/(Cz*Dz + e_v);
  // Horizonal shift
  real_type SHz = mC11*gamma + mC12*Fz + mC13;
  // Vertical shift
  real_type SVz = mC14*Fz + mC15 + (mC16*Fz + mC17)*Fz*gamma;
  // Shifted lateral slip
  real_type alphaz = rad2deg*alpha + SHz;
  // Curvature factor
  real_type Ez = ((mC7*Fz + mC8)*Fz + mC9)
    *(1 - (mC19*gamma + mC20)*copysign(1, alphaz))/(1 - mC10*fabs(gamma));
  // See P175 Note on Fig 4.10: Clamp E <= 1 to avoid unrealistic behaviour
  Ez = min(Ez, real_type(1));
  // The resulting lateral force
  real_type Bza = Bz*alphaz;
  real_type Mz = Dz*sin(Cz*atan(Bza - Ez*(Bza - atan(Bza)))) + SVz;

  return FzRatio*Vector6(Vector3(0, 0, Mz), Vector3(Fx, Fy, 1e3*Fz));
}

void
Pacejka94::setSpringConstant(const real_type& springConstant)
{
  mSpringConstant = springConstant;
}

const real_type&
Pacejka94::getSpringConstant(void) const
{
  return mSpringConstant;
}

void
Pacejka94::setDampingConstant(const real_type& dampingConstant)
{
  mDampingConstant = dampingConstant;
}

const real_type&
Pacejka94::getDampingConstant(void) const
{
  return mDampingConstant;
}

} // namespace OpenFDM
