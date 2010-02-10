/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Bias_H
#define OpenFDM_Bias_H

#include <string>

#include "SimpleDirectModel.h"

namespace OpenFDM {

class Bias : public SimpleDirectModel {
  OPENFDM_OBJECT(Bias, SimpleDirectModel);
public:
  Bias(const std::string& name, const real_type& bias = real_type(0));
  virtual ~Bias(void);

  void output(Context& context) const;

  const Matrix& getBias(void) const;
  void setBias(const Matrix& bias);
  void setBias(const real_type& bias);

private:
  Matrix mBias;
};

} // namespace OpenFDM

#endif
