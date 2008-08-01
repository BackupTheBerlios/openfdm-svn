/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_StateInfo_H
#define OpenFDM_StateInfo_H

#include "Referenced.h"

namespace OpenFDM {

class StateValue;
class LeafContext;

template<typename T>
class TemplateInfoVector;

class StateInfo : public Referenced {
public:
  StateInfo(unsigned index = 0) : mIndex(index)
  { }

  unsigned getIndex() const { return mIndex; }

  // FIXME: Do we need that mechanism to allocate that stuff?
  // May be the Leaf::alloc routine does that well??
  StateValue* newStateValue(const LeafContext& leafContext) const
  { return newStateValueImplementation(leafContext); }

  // Used for trimming, the target function should be zero, if well trimmed
  // virtual real_type trimError(const StateValue&, const StateValue&) const
  // { return 0; }

  static void destroy(StateInfo* stateInfo)
  { delete stateInfo; }

protected:
  virtual ~StateInfo() {}
  virtual StateValue* newStateValueImplementation(const LeafContext&) const = 0;

private:
  template<typename T>
  friend class TemplateInfoVector;
  unsigned mIndex;
};

} // namespace OpenFDM

#endif
