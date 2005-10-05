/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Variant_H
#define OpenFDM_Variant_H

#include <string>

#include "Types.h"
#include "Referenced.h"
#include "RefPtr.h"
#include "Matrix.h"

namespace OpenFDM {

class Variant {
public:
  /// Default constructor
  Variant(void) :
    mData(0)
  {}
  /// Copy constructor
  Variant(const Variant& v) :
    mData(v.mData)
  {}
  /// Create unsigned integer variant
  Variant(const unsigned& value) :
    mData(new UnsignedVariantData(value))
  {}
  /// Create integer variant
  Variant(const int& value) :
    mData(new IntegerVariantData(value))
  {}
  /// Create real scalar variant
  Variant(const real_type& value) :
    mData(new RealVariantData(value))
  {}
  /// Create a matrix/vector variant
  template<typename Impl, LinAlg::size_type m, LinAlg::size_type n>
  Variant(const LinAlg::MatrixRValue<Impl,m,n>& value) :
    mData(new MatrixVariantData(value))
  {}
  /// Create a string variant
  Variant(const std::string& value) :
    mData(new StringVariantData(value))
  {}

  /// Assignment operator
  /// Variant follows the copy on copy sematics, that is in this operator
  /// the contents of the Variant is copied.
  inline Variant& operator=(const Variant& value);

  /// Returns true if the variant contains some value
  inline bool isValid(void) const;

  /// Returns true if the variant can be converted to the given type.
  inline bool isString(void) const;
  inline bool isReal(void) const;
  inline bool isInteger(void) const;
  inline bool isUnsigned(void) const;
  inline bool isMatrix(void) const;

  /// Extraction function.
  std::string toString(void) const;
  /// Extraction function.
  real_type toReal(void) const;
  /// Extraction function.
  int toInteger(void) const;
  /// Extraction function.
  unsigned toUnsigned(void) const;
  /// Extraction function.
  Matrix toMatrix(void) const;

private:

  struct IntegerVariantData;
  struct UnsignedVariantData;
  struct RealVariantData;
  struct MatrixVariantData;
  struct StringVariantData;
  
  struct VariantData : public Referenced {
    VariantData() {}
    virtual ~VariantData() {}
    
    virtual const UnsignedVariantData* toUnsignedVariant(void) const
    { return 0; }
    virtual const IntegerVariantData* toIntegerVariant(void) const
    { return 0; }
    virtual const RealVariantData* toRealVariant(void) const
    { return 0; }
    virtual const MatrixVariantData* toMatrixVariant(void) const
    { return 0; }
    virtual const StringVariantData* toStringVariant(void) const
    { return 0; }
  };
  
  template<typename T>
  struct VariantDataImpl : public VariantData {
    VariantDataImpl(void) {}
    VariantDataImpl(const T& data) : mData(data) {}
    T mData;
  };
  
  struct IntegerVariantData : VariantDataImpl<int> {
    IntegerVariantData(const int& data) { mData = data; }
    virtual const IntegerVariantData* toIntegerVariant(void) const
    { return this; }
  };
  struct UnsignedVariantData : VariantDataImpl<int> {
    UnsignedVariantData(const int& data) { mData = data; }
    virtual const UnsignedVariantData* toUnsignedVariant(void) const
    { return this; }
  };
  struct RealVariantData : VariantDataImpl<real_type> {
    RealVariantData(const real_type& data) { mData = data; }
    virtual const RealVariantData* toRealVariant(void) const
    { return this; }
  };
  struct MatrixVariantData : VariantDataImpl<Matrix> {
    template<typename Impl, LinAlg::size_type m, LinAlg::size_type n>
    MatrixVariantData(const LinAlg::MatrixRValue<Impl,m,n>& data)
    { mData = data; }
    virtual const MatrixVariantData* toMatrixVariant(void) const
    { return this; }
  };
  struct StringVariantData : VariantDataImpl<std::string> {
    StringVariantData(const std::string& data) { mData = data; }
    virtual const StringVariantData* toStringVariant(void) const
    { return this; }
  };
  
  shared_ptr<VariantData> mData;
};

inline Variant&
Variant::operator=(const Variant& value)
{
  mData = value.mData;
  return *this;
}

inline bool
Variant::isValid(void) const
{
  return mData;
}

inline bool
Variant::isString() const
{
  return mData && mData->toStringVariant();
}

inline bool
Variant::isReal() const
{
  return mData && mData->toRealVariant();
}

inline bool
Variant::isInteger() const
{
  return mData && mData->toIntegerVariant();
}

inline bool
Variant::isUnsigned() const
{
  return mData && mData->toUnsignedVariant();
}

inline bool
Variant::isMatrix() const
{
  return mData && mData->toMatrixVariant();
}

} // namespace OpenFDM

#endif
