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

  virtual NumericPortValue* toNumericPortValue() { return this; }
  virtual const NumericPortValue* toNumericPortValue() const { return this; }

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

class RealInputPortHandle {
public:
  RealInputPortHandle(const NumericPortValue* numericPortValue = 0) :
    mNumericPortValue(numericPortValue)
  { }

  bool isConnected() const
  { return mNumericPortValue; }

  operator const real_type&() const
  {
    OpenFDMAssert(mNumericPortValue);
    OpenFDMAssert(size(mNumericPortValue->getValue()) == Size(1, 1));
    return mNumericPortValue->getValue()(0, 0);
  }

private: 
  SharedPtr<const NumericPortValue> mNumericPortValue;
};

class RealOutputPortHandle {
public:
  RealOutputPortHandle(NumericPortValue* numericPortValue = 0) :
    mNumericPortValue(numericPortValue)
  { }

  bool isConnected() const
  { return mNumericPortValue; }

  operator const real_type&() const
  {
    OpenFDMAssert(mNumericPortValue);
    OpenFDMAssert(size(mNumericPortValue->getValue()) == Size(1, 1));
    return mNumericPortValue->getValue()(0, 0);
  }
  operator real_type&()
  {
    OpenFDMAssert(mNumericPortValue);
    OpenFDMAssert(size(mNumericPortValue->getValue()) == Size(1, 1));
    return mNumericPortValue->getValue()(0, 0);
  }
  RealOutputPortHandle& operator=(const real_type& value)
  {
    OpenFDMAssert(mNumericPortValue);
    OpenFDMAssert(size(mNumericPortValue->getValue()) == Size(1, 1));
    mNumericPortValue->getValue()(0, 0) = value;
    return *this;
  }

private: 
  SharedPtr<NumericPortValue> mNumericPortValue;
};

class MatrixInputPortHandle {
public:
  MatrixInputPortHandle(const NumericPortValue* numericPortValue = 0) :
    mNumericPortValue(numericPortValue)
  { }

  bool isConnected() const
  { return mNumericPortValue; }

  operator const Matrix&() const
  {
    OpenFDMAssert(mNumericPortValue);
    return mNumericPortValue->getValue();
  }

private: 
  SharedPtr<const NumericPortValue> mNumericPortValue;
};

class MatrixOutputPortHandle {
public:
  MatrixOutputPortHandle(NumericPortValue* numericPortValue = 0) :
    mNumericPortValue(numericPortValue)
  { }

  bool isConnected() const
  { return mNumericPortValue; }

  operator const Matrix&() const
  {
    OpenFDMAssert(mNumericPortValue);
    return mNumericPortValue->getValue();
  }
  operator Matrix&()
  {
    OpenFDMAssert(mNumericPortValue);
    return mNumericPortValue->getValue();
  }
  template<typename Impl, LinAlg::size_type m, LinAlg::size_type n>
  MatrixOutputPortHandle& operator=(const LinAlg::MatrixRValue<Impl,m,n>& value)
  {
    OpenFDMAssert(mNumericPortValue);
    OpenFDMAssert(size(mNumericPortValue->getValue()) == Size(value));
    mNumericPortValue->getValue() = value;
    return *this;
  }

private: 
  SharedPtr<NumericPortValue> mNumericPortValue;
};

} // namespace OpenFDM

#endif
