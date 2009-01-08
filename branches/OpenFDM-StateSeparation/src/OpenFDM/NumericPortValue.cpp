/* -*-c++-*- OpenFDM - Copyright (C) 2007-2009 Mathias Froehlich 
 *
 */

#include "NumericPortValue.h"

namespace OpenFDM {

NumericPortValue::NumericPortValue(const Size& size) :
  mMatrix(size(0), size(1))
{
  mMatrix.clear();
}

NumericPortValue::~NumericPortValue()
{
}

} // namespace OpenFDM
