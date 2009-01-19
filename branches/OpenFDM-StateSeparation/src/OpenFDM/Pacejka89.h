/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Pacejka89_H
#define OpenFDM_Pacejka89_H

#include "PacejkaTire.h"

namespace OpenFDM {

class Pacejka89 : public PacejkaTire {
  OPENFDM_OBJECT(Pacejka89, PacejkaTire);
public:
  Pacejka89(const std::string& name);
  virtual ~Pacejka89(void);

  // Helper function to compute the resulting force
  virtual Vector6 getForce(const real_type& rho, const real_type& rhoDot,
                           const real_type& alpha, const real_type& kappa,
                           const real_type& gamma, const real_type& phi) const;

  void setSpringConstant(const real_type& springConstant);
  const real_type& getSpringConstant(void) const;

  void setDampingConstant(const real_type& dampingConstant);
  const real_type& getDampingConstant(void) const;

  const real_type& getFzMin() const { return mFzMin; }
  void setFzMin(const real_type& value) { mFzMin = value; }

  const real_type& getA0() const { return mA0; }
  void setA0(const real_type& value) { mA0 = value; }

  const real_type& getA1() const { return mA1; }
  void setA1(const real_type& value) { mA1 = value; }

  const real_type& getA2() const { return mA2; }
  void setA2(const real_type& value) { mA2 = value; }

  const real_type& getA3() const { return mA3; }
  void setA3(const real_type& value) { mA3 = value; }

  const real_type& getA4() const { return mA4; }
  void setA4(const real_type& value) { mA4 = value; }

  const real_type& getA5() const { return mA5; }
  void setA5(const real_type& value) { mA5 = value; }

  const real_type& getA6() const { return mA6; }
  void setA6(const real_type& value) { mA6 = value; }

  const real_type& getA7() const { return mA7; }
  void setA7(const real_type& value) { mA7 = value; }

  const real_type& getA8() const { return mA8; }
  void setA8(const real_type& value) { mA8 = value; }

  const real_type& getA9() const { return mA9; }
  void setA9(const real_type& value) { mA9 = value; }

  const real_type& getA10() const { return mA10; }
  void setA10(const real_type& value) { mA10 = value; }

  const real_type& getA11() const { return mA11; }
  void setA11(const real_type& value) { mA11 = value; }

  const real_type& getA12() const { return mA12; }
  void setA12(const real_type& value) { mA12 = value; }

  const real_type& getA13() const { return mA13; }
  void setA13(const real_type& value) { mA13 = value; }


  const real_type& getB0() const { return mB0; }
  void setB0(const real_type& value) { mB0 = value; }

  const real_type& getB1() const { return mB1; }
  void setB1(const real_type& value) { mB1 = value; }

  const real_type& getB2() const { return mB2; }
  void setB2(const real_type& value) { mB2 = value; }

  const real_type& getB3() const { return mB3; }
  void setB3(const real_type& value) { mB3 = value; }

  const real_type& getB4() const { return mB4; }
  void setB4(const real_type& value) { mB4 = value; }

  const real_type& getB5() const { return mB5; }
  void setB5(const real_type& value) { mB5 = value; }

  const real_type& getB6() const { return mB6; }
  void setB6(const real_type& value) { mB6 = value; }

  const real_type& getB7() const { return mB7; }
  void setB7(const real_type& value) { mB7 = value; }

  const real_type& getB8() const { return mB8; }
  void setB8(const real_type& value) { mB8 = value; }

  const real_type& getB9() const { return mB9; }
  void setB9(const real_type& value) { mB9 = value; }

  const real_type& getB10() const { return mB10; }
  void setB10(const real_type& value) { mB10 = value; }


  const real_type& getC0() const { return mC0; }
  void setC0(const real_type& value) { mC0 = value; }

  const real_type& getC1() const { return mC1; }
  void setC1(const real_type& value) { mC1 = value; }

  const real_type& getC2() const { return mC2; }
  void setC2(const real_type& value) { mC2 = value; }

  const real_type& getC3() const { return mC3; }
  void setC3(const real_type& value) { mC3 = value; }

  const real_type& getC4() const { return mC4; }
  void setC4(const real_type& value) { mC4 = value; }

  const real_type& getC5() const { return mC5; }
  void setC5(const real_type& value) { mC5 = value; }

  const real_type& getC6() const { return mC6; }
  void setC6(const real_type& value) { mC6 = value; }

  const real_type& getC7() const { return mC7; }
  void setC7(const real_type& value) { mC7 = value; }

  const real_type& getC8() const { return mC8; }
  void setC8(const real_type& value) { mC8 = value; }

  const real_type& getC9() const { return mC9; }
  void setC9(const real_type& value) { mC9 = value; }

  const real_type& getC10() const { return mC10; }
  void setC10(const real_type& value) { mC10 = value; }

  const real_type& getC11() const { return mC11; }
  void setC11(const real_type& value) { mC11 = value; }

  const real_type& getC12() const { return mC12; }
  void setC12(const real_type& value) { mC12 = value; }

  const real_type& getC13() const { return mC13; }
  void setC13(const real_type& value) { mC13 = value; }

  const real_type& getC14() const { return mC14; }
  void setC14(const real_type& value) { mC14 = value; }

  const real_type& getC15() const { return mC15; }
  void setC15(const real_type& value) { mC15 = value; }

  const real_type& getC16() const { return mC16; }
  void setC16(const real_type& value) { mC16 = value; }

  const real_type& getC17() const { return mC17; }
  void setC17(const real_type& value) { mC17 = value; }

private:
  real_type mSpringConstant;
  real_type mDampingConstant;

  real_type mFzMin;

  real_type mA0;
  real_type mA1, mA2;
  real_type mA3, mA4, mA5;
  real_type mA6, mA7;
  real_type mA8, mA9, mA10;
  real_type mA11, mA12, mA13;

  real_type mB0;
  real_type mB1, mB2;
  real_type mB3, mB4, mB5;
  real_type mB6, mB7, mB8;
  real_type mB9, mB10;

  real_type mC0;
  real_type mC1, mC2;
  real_type mC3, mC4, mC5, mC6;
  real_type mC7, mC8, mC9, mC10;
  real_type mC11, mC12, mC13;
  real_type mC14, mC15, mC16, mC17;
};

} // namespace OpenFDM

#endif
