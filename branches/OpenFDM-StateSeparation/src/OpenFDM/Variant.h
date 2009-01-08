/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Variant_H
#define OpenFDM_Variant_H

#include <string>

#include "Types.h"
#include "Referenced.h"
#include "SharedPtr.h"
#include "Matrix.h"
#include "TableData.h"

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

  // FIXME: just to hack reflection for now
  Variant(const BreakPointVector& value) :
    mData(new BreakPointVectorVariantData(value))
  {}
  Variant(const TableData<1>& value) :
    mData(new Table1DVariantData(value))
  {}
  Variant(const TableData<2>& value) :
    mData(new Table2DVariantData(value))
  {}
  Variant(const TableData<3>& value) :
    mData(new Table3DVariantData(value))
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

  // FIXME: just to hack reflection for now
  inline bool isBreakPointVector(void) const;
  inline bool isTable1D(void) const;
  inline bool isTable2D(void) const;
  inline bool isTable3D(void) const;

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

  // FIXME: just to hack reflection for now
  BreakPointVector toBreakPointVector(void) const;
  TableData<1> toTable1D(void) const;
  TableData<2> toTable2D(void) const;
  TableData<3> toTable3D(void) const;

private:

  struct IntegerVariantData;
  struct UnsignedVariantData;
  struct RealVariantData;
  struct MatrixVariantData;
  struct StringVariantData;

  struct BreakPointVectorVariantData;
  struct Table1DVariantData;
  struct Table2DVariantData;
  struct Table3DVariantData;
  
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

    virtual const BreakPointVectorVariantData* toBreakPointVectorVariant(void) const
    { return 0; }
    virtual const Table1DVariantData* toTable1DVariant(void) const
    { return 0; }
    virtual const Table2DVariantData* toTable2DVariant(void) const
    { return 0; }
    virtual const Table3DVariantData* toTable3DVariant(void) const
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
  struct UnsignedVariantData : VariantDataImpl<unsigned> {
    UnsignedVariantData(const unsigned& data) { mData = data; }
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

  struct BreakPointVectorVariantData : VariantDataImpl<BreakPointVector> {
    BreakPointVectorVariantData(const BreakPointVector& data) { mData = data; }
    virtual const BreakPointVectorVariantData* toBreakPointVectorVariant(void) const
    { return this; }
  };
  struct Table1DVariantData : VariantDataImpl<TableData<1> > {
    Table1DVariantData(const TableData<1>& data) { mData = data; }
    virtual const Table1DVariantData* toTable1DVariant(void) const
    { return this; }
  };
  struct Table2DVariantData : VariantDataImpl<TableData<2> > {
    Table2DVariantData(const TableData<2>& data) { mData = data; }
    virtual const Table2DVariantData* toTable2DVariant(void) const
    { return this; }
  };
  struct Table3DVariantData : VariantDataImpl<TableData<3> > {
    Table3DVariantData(const TableData<3>& data) { mData = data; }
    virtual const Table3DVariantData* toTable3DVariant(void) const
    { return this; }
  };
  
  SharedPtr<VariantData> mData;
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

inline bool
Variant::isBreakPointVector() const
{
  return mData && mData->toBreakPointVectorVariant();
}

inline bool
Variant::isTable1D() const
{
  return mData && mData->toTable1DVariant();
}

inline bool
Variant::isTable2D() const
{
  return mData && mData->toTable2DVariant();
}

inline bool
Variant::isTable3D() const
{
  return mData && mData->toTable3DVariant();
}

/// Hmmm, .... FIXME
template<typename T>
inline void
variant_copy(const Variant& variant, T& value)
{}

template<>
inline void
variant_copy(const Variant& variant, std::string& value)
{ value = variant.toString(); }

template<>
inline void
variant_copy(const Variant& variant, real_type& value)
{ value = variant.toReal(); }

template<>
inline void
variant_copy(const Variant& variant, int& value)
{ value = variant.toInteger(); }

template<>
inline void
variant_copy(const Variant& variant, unsigned& value)
{ value = variant.toUnsigned(); }

template<typename Impl, LinAlg::size_type m, LinAlg::size_type n> 
inline void
variant_copy(const Variant& variant, LinAlg::MatrixLValue<Impl,m,n>& value)
{ value = variant.toMatrix(); }

template<> 
inline void
variant_copy(const Variant& variant, BreakPointVector& value)
{ value = variant.toBreakPointVector(); }

template<> 
inline void
variant_copy(const Variant& variant, TableData<1>& value)
{ value = variant.toTable1D(); }
template<> 
inline void
variant_copy(const Variant& variant, TableData<2>& value)
{ value = variant.toTable2D(); }
template<> 
inline void
variant_copy(const Variant& variant, TableData<3>& value)
{ value = variant.toTable3D(); }

} // namespace OpenFDM

#endif
