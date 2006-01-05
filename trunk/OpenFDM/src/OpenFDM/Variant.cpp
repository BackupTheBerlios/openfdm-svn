/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include <sstream>

#include "Types.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Variant.h"

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

} // namespace OpenFDM
