/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MatrixStateValue_H
#define OpenFDM_MatrixStateValue_H

#include "ContinousStateValue.h"
#include "Matrix.h"

namespace OpenFDM {

class MatrixStateValue : public ContinousStateValue {
public:
  MatrixStateValue(const Size& size) : mMatrix(size)
  { }
  const Matrix& getMatrix() const
  { return mMatrix; }
  Matrix& getMatrix()
  { return mMatrix; }
  void setMatrix(const Matrix& matrix)
  { OpenFDMAssert(size(matrix) == size(mMatrix)); mMatrix = matrix; }

protected:
  virtual ~MatrixStateValue();

private:
  Matrix mMatrix;
};

} // namespace OpenFDM

#endif
