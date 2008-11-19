/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_UnaryFunction_H
#define OpenFDM_UnaryFunction_H

#include <string>
#include "UnaryModel.h"

namespace OpenFDM {

/// Class representing a model with exactly one input.
class UnaryFunction : public UnaryModel {
  OPENFDM_OBJECT(UnaryFunction, UnaryModel);
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
    Sin,
    Sqr,
    Sqrt,
    Tan
  };

  UnaryFunction(const std::string& name, const Type& type);
  virtual ~UnaryFunction(void);

  ModelContext* newModelContext(PortValueList&) const;
  void output(const Matrix& inputValue, Matrix& outputValue) const;

  void setType(const Type& type);
  const Type& getType(void) const;

private:
  Type mType;
};

} // namespace OpenFDM

#endif
