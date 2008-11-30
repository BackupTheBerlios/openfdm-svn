/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich
 *
 */

#ifndef OpenFDM_BreakPointLookup_H
#define OpenFDM_BreakPointLookup_H

#include "TableData.h"
#include "SimpleDirectModel.h"

namespace OpenFDM {

class BreakPointLookup : public SimpleDirectModel {
  OPENFDM_OBJECT(BreakPointLookup, SimpleDirectModel);
public:
  BreakPointLookup(const std::string& name);
  virtual ~BreakPointLookup(void);

  virtual void output(Context& context) const;

  void setBreakPointVector(const BreakPointVector& breakPointVector)
  { mBreakPointVector = breakPointVector; }
  const BreakPointVector& getBreakPointVector(void) const
  { return mBreakPointVector; }

private:
  BreakPointVector mBreakPointVector;
};

} // namespace OpenFDM

#endif
