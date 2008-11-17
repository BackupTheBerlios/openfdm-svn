/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Bias_H
#define OpenFDM_Bias_H

#include <string>

#include "UnaryModel.h"

namespace OpenFDM {

class Bias : public UnaryModel {
  OPENFDM_OBJECT(Bias, UnaryModel);
public:
  Bias(const std::string& name);
  virtual ~Bias(void);

  ModelContext* newModelContext(PortValueList&) const;

  bool alloc(LeafContext& context) const;
  void output(const Matrix& inputValue, Matrix& outputValue) const;

  const Matrix& getBias(void) const;
  void setBias(const Matrix& bias);

private:
  Matrix mBias;
};

} // namespace OpenFDM

#endif
