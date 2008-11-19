/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "UnitConversion.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(UnitConversion, UnaryModel)
  END_OPENFDM_OBJECT_DEF

UnitConversion::UnitConversion(const std::string& name, const Type& type,
                               const Unit& unit) :
  UnaryModel(name),
  mType(type),
  mUnit(unit)
{
}

UnitConversion::~UnitConversion(void)
{
}

ModelContext*
UnitConversion::newModelContext(PortValueList& portValueList) const
{
  return UnaryModel::newModelContext(this, portValueList);
}

void
UnitConversion::output(const Matrix& inputValue, Matrix& outputValue) const
{
  if (mType == UnitToBaseUnit) {
    outputValue = mUnit.convertFrom(inputValue);
  } else {
    outputValue = mUnit.convertTo(inputValue);
  }
}

void
UnitConversion::setType(const UnitConversion::Type& type)
{
  mType = type;
}

const UnitConversion::Type&
UnitConversion::getType(void) const
{
  return mType;
}

void
UnitConversion::setUnit(const Unit& unit)
{
  mUnit = unit;
}

const Unit&
UnitConversion::getUnit(void) const
{
  return mUnit;
}

} // namespace OpenFDM
