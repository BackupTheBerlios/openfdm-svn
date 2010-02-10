/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ComparisonOperator_H
#define OpenFDM_ComparisonOperator_H

#include <string>
#include "SimpleDirectModel.h"

namespace OpenFDM {

class ComparisonOperator : public SimpleDirectModel {
  OPENFDM_OBJECT(ComparisonOperator, SimpleDirectModel);
public:
  enum Type {
    Equal,
    NotEqual,
    Less,
    LessEqual,
    Greater,
    GreaterEqual
  };

  ComparisonOperator(const std::string& name, Type type);
  virtual ~ComparisonOperator(void);

  virtual void output(Context& context) const;

  void setType(const Type& type);
  const Type& getType(void) const;

private:
  Type mType;
};

} // namespace OpenFDM

#endif
