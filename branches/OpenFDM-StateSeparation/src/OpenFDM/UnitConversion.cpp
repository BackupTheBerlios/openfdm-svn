/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "UnitConversion.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(UnitConversion, SimpleDirectModel)
  END_OPENFDM_OBJECT_DEF

UnitConversion::UnitConversion(const std::string& name, const Type& type,
                               const Unit& unit) :
  SimpleDirectModel(name),
  mType(type),
  mUnit(unit)
{
  addInputPort("input");
}

UnitConversion::~UnitConversion(void)
{
}

void
UnitConversion::output(Context& context) const
{
  if (mType == UnitToBaseUnit) {
    context.getOutputValue() = mUnit.convertFrom(context.getInputValue(0));
  } else {
    context.getOutputValue() = mUnit.convertTo(context.getInputValue(0));
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
