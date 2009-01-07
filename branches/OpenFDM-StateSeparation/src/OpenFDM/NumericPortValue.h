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

  const real_type& getValue() const
  {
    OpenFDMAssert(mNumericPortValue);
    OpenFDMAssert(size(mNumericPortValue->getValue()) == Size(1, 1));
    return mNumericPortValue->getValue()(0, 0);
  }

  operator const real_type&() const
  { return getValue(); }

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

  const real_type& getValue() const
  {
    OpenFDMAssert(mNumericPortValue);
    OpenFDMAssert(size(mNumericPortValue->getValue()) == Size(1, 1));
    return mNumericPortValue->getValue()(0, 0);
  }
  real_type& getValue()
  {
    OpenFDMAssert(mNumericPortValue);
    OpenFDMAssert(size(mNumericPortValue->getValue()) == Size(1, 1));
    return mNumericPortValue->getValue()(0, 0);
  }

  operator const real_type&() const
  { return getValue(); }
  operator real_type&()
  { return getValue(); }
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

  const Matrix& getValue() const
  {
    OpenFDMAssert(mNumericPortValue);
    return mNumericPortValue->getValue();
  }

  operator const Matrix&() const
  { return getValue(); }

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

  const Matrix& getValue() const
  {
    OpenFDMAssert(mNumericPortValue);
    return mNumericPortValue->getValue();
  }
  Matrix& getValue()
  {
    OpenFDMAssert(mNumericPortValue);
    return mNumericPortValue->getValue();
  }


  operator const Matrix&() const
  { return getValue(); }
  operator Matrix&()
  { return getValue(); }
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
