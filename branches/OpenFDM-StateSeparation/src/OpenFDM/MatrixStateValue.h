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
  MatrixStateValue()
  { }

  void resize(const Size& size)
  { mMatrix.resize(size); }

  const Matrix& getMatrix() const
  { return mMatrix; }
  Matrix& getMatrix()
  { return mMatrix; }
  void setMatrix(const Matrix& matrix)
  { OpenFDMAssert(size(matrix) == size(mMatrix)); mMatrix = matrix; }

  virtual void setValue(const StateStream& stateStream)
  { stateStream.readSubState(mMatrix); }
  virtual void getValue(StateStream& stateStream) const
  { stateStream.writeSubState(mMatrix); }
  virtual LinAlg::size_type getNumStates() const
  { return rows(mMatrix)*cols(mMatrix); }

protected:
  virtual ~MatrixStateValue();

private:
  Matrix mMatrix;
};

} // namespace OpenFDM

#endif
