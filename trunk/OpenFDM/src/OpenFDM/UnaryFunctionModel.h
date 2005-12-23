/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_UnaryFunctionModel_H
#define OpenFDM_UnaryFunctionModel_H

#include <string>

#include "Types.h"
#include "Model.h"

namespace OpenFDM {

class UnaryFunctionModelImpl;

/// Class representing a model with exactly one input.
class UnaryFunctionModel :
    public Model {
public:
  enum Type {
    Abs,
    Acos,
    Asin,
    Atan,
    Ceil,
    Cos,
    Exp,
    Floor,
    Log,
    Log10,
    Minus,
    Sqr,
    Sqrt,
    Tan
  };

  UnaryFunctionModel(const std::string& name, Type type);
  virtual ~UnaryFunctionModel(void);

  virtual bool init(void);
  virtual void output(const TaskInfo&);

  const real_type& getFunctionValue(void) const;

  void setType(Type type);
  Type getType(void) const;

private:
  SharedPtr<UnaryFunctionModelImpl> mImpl;
  Type mType;
  real_type mFunctionValue;
};

class UnitConversionModel :
    public Model {
public:
  enum Type {
    UnitToSi,
    SiToUnit
  };

  UnitConversionModel(const std::string& name, Type type, Unit unit);
  virtual ~UnitConversionModel(void);

  virtual bool init(void);
  virtual void output(const TaskInfo&);

  const real_type& getFunctionValue(void) const;

  void setType(Type type);
  Type getType(void) const;

  void setUnit(Unit unit);
  Unit getUnit(void) const;

private:
  Type mType;
  Unit mUnit;
  real_type mValue;
};

} // namespace OpenFDM

#endif
