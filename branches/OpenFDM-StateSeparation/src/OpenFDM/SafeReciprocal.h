/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_SafeReciprocal_H
#define OpenFDM_SafeReciprocal_H

#include <string>
#include "SimpleDirectModel.h"

namespace OpenFDM {

class SafeReciprocal : public SimpleDirectModel {
  OPENFDM_OBJECT(SafeReciprocal, SimpleDirectModel);
public:
  SafeReciprocal(const std::string& name,
                 const real_type& epsilon = Limits<real_type>::safe_min());
  virtual ~SafeReciprocal(void);

  void output(Context& context) const;

  const real_type& getEpsilon() const
  { return mEpsilon; }
  void setEpsilon(const real_type& epsilon)
  { mEpsilon = epsilon; }

private:
  real_type mEpsilon;
};

} // namespace OpenFDM

#endif
