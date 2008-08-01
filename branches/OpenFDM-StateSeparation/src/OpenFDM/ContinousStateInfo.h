/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ContinousStateInfo_H
#define OpenFDM_ContinousStateInfo_H

#include "ContinousStateValue.h"
#include "StateInfo.h"

namespace OpenFDM {

class LeafContext;

class ContinousStateInfo : public StateInfo {
public:
  ContinousStateValue* newStateValue(const LeafContext& leafContext) const
  { return newStateValueImplementation(leafContext); }

protected:
  virtual ~ContinousStateInfo() {}
  virtual ContinousStateValue* newStateValueImplementation(const LeafContext&) const = 0;
};

} // namespace OpenFDM

#endif
