/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "UnaryFunction.h"

#include <string>

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(UnaryFunction, UnaryModel)
  END_OPENFDM_OBJECT_DEF

UnaryFunction::UnaryFunction(const std::string& name, const Type& type) :
  UnaryModel(name),
  mType(type)
{
}

UnaryFunction::~UnaryFunction(void)
{
}

ModelContext*
UnaryFunction::newModelContext(PortValueList& portValueList) const
{
  return UnaryModel::newModelContext(this, portValueList);
}

void
UnaryFunction::output(const Matrix& inputValue, Matrix& outputValue) const
{
  // FIXME, optimize, move that into a proper context ...
  // For now make it work
  Size sz = size(inputValue);
  for (unsigned j = 0; j < sz(1); ++j) {
    for (unsigned i = 0; i < sz(0); ++i) {
      switch (mType) {
      case Abs:
        outputValue(i, j) = fabs(inputValue(i, j));
        break;
      case Acos:
        outputValue(i, j) = acos(inputValue(i, j));
        break;
      case Asin:
        outputValue(i, j) = asin(inputValue(i, j));
        break;
      case Atan:
        outputValue(i, j) = atan(inputValue(i, j));
        break;
      case Ceil:
        outputValue(i, j) = ceil(inputValue(i, j));
        break;
      case Cos:
        outputValue(i, j) = cos(inputValue(i, j));
        break;
      case Exp:
        outputValue(i, j) = exp(inputValue(i, j));
        break;
      case Floor:
        outputValue(i, j) = floor(inputValue(i, j));
        break;
      case Log:
        outputValue(i, j) = log(inputValue(i, j));
        break;
      case Log10:
        outputValue(i, j) = log10(inputValue(i, j));
        break;
      case Minus:
        outputValue(i, j) = -inputValue(i, j);
        break;
      case Sin:
        outputValue(i, j) = sin(inputValue(i, j));
        break;
      case Sqr:
        outputValue(i, j) = inputValue(i, j)*inputValue(i, j);
        break;
      case Sqrt:
        outputValue(i, j) = sqrt(inputValue(i, j));
        break;
      case Tan:
        outputValue(i, j) = tan(inputValue(i, j));
        break;
      default:
        outputValue(i, j) = inputValue(i, j);
        break;
      }
    }
  }
}

void
UnaryFunction::setType(const UnaryFunction::Type& type)
{
  mType = type;
}

const UnaryFunction::Type&
UnaryFunction::getType(void) const
{
  return mType;
}

} // namespace OpenFDM
