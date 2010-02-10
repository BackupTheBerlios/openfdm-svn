/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_LogicalOperator_H
#define OpenFDM_LogicalOperator_H

#include <string>
#include "SimpleDirectModel.h"

namespace OpenFDM {

class LogicalOperator : public SimpleDirectModel {
  OPENFDM_OBJECT(LogicalOperator, SimpleDirectModel);
public:
  enum Type {
    LogicalAND,
    LogicalOR,
    LogicalNAND,
    LogicalNOR,
    LogicalXOR,
    LogicalNOT
  };

  LogicalOperator(const std::string& name, Type type);
  virtual ~LogicalOperator(void);

  virtual void output(Context& context) const;

  void setType(const Type& type);
  const Type& getType(void) const;

private:
  Type mType;
};

} // namespace OpenFDM

#endif
