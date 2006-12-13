/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Variant.h"

#include <sstream>

#include "Types.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"

namespace OpenFDM {

std::string
Variant::toString() const
{
  if (mData) {
    const StringVariantData* stringData = mData->toStringVariant();
    if (stringData)
      return stringData->mData;

    const RealVariantData* realData = mData->toRealVariant();
    if (realData) {
      std::stringstream sstr;
      sstr << realData->mData;
      return sstr.str();
    }

    const IntegerVariantData* integerData = mData->toIntegerVariant();
    if (integerData) {
      std::stringstream sstr;
      sstr << integerData->mData;
      return sstr.str();
    }

    const UnsignedVariantData* unsignedData = mData->toUnsignedVariant();
    if (unsignedData) {
      std::stringstream sstr;
      sstr << unsignedData->mData;
      return sstr.str();
    }

    const MatrixVariantData* matrixData = mData->toMatrixVariant();
    if (matrixData) {
      std::stringstream sstr;
      sstr << matrixData->mData;
      return sstr.str();
    }

    const TableLookupVariantData* tableLookupData = mData->toTableLookupVariant();
    if (tableLookupData) {
      std::stringstream sstr;
      sstr << tableLookupData->mData;
      return sstr.str();
    }

    const Table1DVariantData* table1DData = mData->toTable1DVariant();
    if (table1DData) {
      std::stringstream sstr;
      sstr << table1DData->mData;
      return sstr.str();
    }

    const Table2DVariantData* table2DData = mData->toTable2DVariant();
    if (table2DData) {
      std::stringstream sstr;
      sstr << table2DData->mData;
      return sstr.str();
    }

    const Table3DVariantData* table3DData = mData->toTable3DVariant();
    if (table3DData) {
      std::stringstream sstr;
      sstr << table3DData->mData;
      return sstr.str();
    }
  }

  return std::string();
}

real_type
Variant::toReal() const
{
  if (mData) {
    const RealVariantData* realData = mData->toRealVariant();
    if (realData)
      return realData->mData;

    const IntegerVariantData* integerData = mData->toIntegerVariant();
    if (integerData)
      return integerData->mData;

    const UnsignedVariantData* unsignedData = mData->toUnsignedVariant();
    if (unsignedData)
      return unsignedData->mData;

    const MatrixVariantData* matrixData = mData->toMatrixVariant();
    if (matrixData &&
        matrixData->mData.rows() == 1 && matrixData->mData.cols() == 1)
      return matrixData->mData(1,1);
  }

  return 0;
}

int
Variant::toInteger() const
{
  if (mData) {
    const IntegerVariantData* integerData = mData->toIntegerVariant();
    if (integerData)
      return integerData->mData;

    const UnsignedVariantData* unsignedData = mData->toUnsignedVariant();
    if (unsignedData)
      return static_cast<int>(unsignedData->mData);

    const RealVariantData* realData = mData->toRealVariant();
    if (realData)
      return static_cast<int>(floor(realData->mData));

    const MatrixVariantData* matrixData = mData->toMatrixVariant();
    if (matrixData &&
        matrixData->mData.rows() == 1 && matrixData->mData.cols() == 1)
      return static_cast<int>(floor(matrixData->mData(1,1)));
  }

  return 0;
}

unsigned
Variant::toUnsigned() const
{
  if (mData) {
    const UnsignedVariantData* unsignedData = mData->toUnsignedVariant();
    if (unsignedData)
      return unsignedData->mData;
  }

  return 0;
}

Matrix
Variant::toMatrix() const
{
  if (mData) {
    const MatrixVariantData* matrixData = mData->toMatrixVariant();
    if (matrixData)
      return matrixData->mData;

    Matrix m(1, 1);
    const RealVariantData* realData = mData->toRealVariant();
    if (realData) {
      m(1, 1) = realData->mData;
      return m;
    }

    const IntegerVariantData* integerData = mData->toIntegerVariant();
    if (integerData) {
      m(1, 1) = integerData->mData;
      return m;
    }

    const UnsignedVariantData* unsignedData = mData->toUnsignedVariant();
    if (unsignedData) {
      m(1, 1) = unsignedData->mData;
      return m;
    }
  }

  return Matrix();
}

TableLookup
Variant::toTableLookup(void) const
{
  const TableLookupVariantData* data = mData->toTableLookupVariant();
  if (data)
    return data->mData;
  return TableLookup();
}

TableData<1>
Variant::toTable1D(void) const
{
  const Table1DVariantData* data = mData->toTable1DVariant();
  if (data)
    return data->mData;
  return TableData<1>();
}

TableData<2>
Variant::toTable2D(void) const
{
  const Table2DVariantData* data = mData->toTable2DVariant();
  if (data)
    return data->mData;
  return TableData<2>();
}

TableData<3>
Variant::toTable3D(void) const
{
  const Table3DVariantData* data = mData->toTable3DVariant();
  if (data)
    return data->mData;
  return TableData<3>();
}

} // namespace OpenFDM
