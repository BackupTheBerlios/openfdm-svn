/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_NumericPortValue_H
#define OpenFDM_NumericPortValue_H

#include "Matrix.h"
#include "PortValue.h"

namespace OpenFDM {

class NumericPortValue : public PortValue {
public:
  NumericPortValue(const Size& size);
  virtual ~NumericPortValue();

  const Matrix& getValue() const
  { return mMatrix; }
  Matrix& getValue()
  { return mMatrix; }
  void setValue(const Matrix& matrix)
  {
    OpenFDMAssert(size(matrix) == size(mMatrix));
    mMatrix = matrix;
  }

private:
  Matrix mMatrix;
};

} // namespace OpenFDM

#endif
