/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_UnitConversion_H
#define OpenFDM_UnitConversion_H

#include <string>

#include "Unit.h"
#include "SimpleDirectModel.h"

namespace OpenFDM {

class UnitConversion : public SimpleDirectModel {
  OPENFDM_OBJECT(UnitConversion, SimpleDirectModel);
public:
  enum Type {
    UnitToBaseUnit,
    BaseUnitToUnit
  };

  UnitConversion(const std::string& name, const Type& type, const Unit& unit);
  virtual ~UnitConversion(void);

  void output(Context& context) const;

  void setType(const Type& type);
  const Type& getType(void) const;

  void setUnit(const Unit& unit);
  const Unit& getUnit(void) const;

private:
  Type mType;
  Unit mUnit;
};

} // namespace OpenFDM

#endif
