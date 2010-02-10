/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_PacejkaSimple_H
#define OpenFDM_PacejkaSimple_H

#include "PacejkaTire.h"

namespace OpenFDM {

class PacejkaSimple : public PacejkaTire {
  OPENFDM_OBJECT(PacejkaSimple, PacejkaTire);
public:
  PacejkaSimple(const std::string& name);
  virtual ~PacejkaSimple(void);

  // Helper function to compute the resulting force
  virtual Vector6 getForce(const real_type& rho, const real_type& rhoDot,
                           const real_type& alpha, const real_type& kappa,
                           const real_type& gamma, const real_type& phi) const;

  void setSpringConstant(const real_type& springConstant);
  const real_type& getSpringConstant(void) const;

  void setDampingConstant(const real_type& dampingConstant);
  const real_type& getDampingConstant(void) const;

  const real_type& getBx() const { return mBx; }
  void setBx(const real_type& value) { mBx = value; }

  const real_type& getCx() const { return mCx; }
  void setCx(const real_type& value) { mCx = value; }

  const real_type& getDx() const { return mDx; }
  void setDx(const real_type& value) { mDx = value; }

  const real_type& getEx() const { return mEx; }
  void setEx(const real_type& value) { mEx = value; }


  const real_type& getBy() const { return mBy; }
  void setBy(const real_type& value) { mBy = value; }

  const real_type& getCy() const { return mCy; }
  void setCy(const real_type& value) { mCy = value; }

  const real_type& getDy() const { return mDy; }
  void setDy(const real_type& value) { mDy = value; }

  const real_type& getEy() const { return mEy; }
  void setEy(const real_type& value) { mEy = value; }

private:
  real_type mSpringConstant;
  real_type mDampingConstant;

  real_type mBx, mCx, mDx, mEx;
  real_type mBy, mCy, mDy, mEy;
};

} // namespace OpenFDM

#endif
