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
  ContinousStateInfo(bool isTrimable = false) :
    mRelTol(1e-3),
    mAbsTol(1e-5),
    mIsTrimable(isTrimable)
  { }

  void setIsTrimable(bool isTrimable)
  { mIsTrimable = isTrimable; }
  bool getIsTrimable() const
  { return mIsTrimable; }

  void setRelTol(const real_type& relTol)
  { mRelTol = relTol; }
  const real_type& getRelTol() const
  { return mRelTol; }

  void setAbsTol(const real_type& absTol)
  { mAbsTol = absTol; }
  const real_type& getAbsTol() const
  { return mAbsTol; }

  ContinousStateValue* newStateValue(const LeafContext& leafContext) const
  { return newStateValueImplementation(leafContext); }

protected:
  virtual ~ContinousStateInfo() {}
  virtual ContinousStateValue* newStateValueImplementation(const LeafContext&) const = 0;
private:
  real_type mRelTol;
  real_type mAbsTol;
  bool mIsTrimable;
//   unsigned char mErrorIndex;
//   unsigned char mDifferentialEquationOrder;
};

} // namespace OpenFDM

#endif
