/* -*-c++-*- OpenFDM - Copyright (C) 2007-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_TemplateValueVector_H
#define OpenFDM_TemplateValueVector_H

#include <vector>
#include "SharedPtr.h"

namespace OpenFDM {

class LeafContext;

template<typename Info, typename Value>
class TemplateValueVector {
public:
  Value* getValue(const Info& info)
  {
    if (mValueVector.size() <= info.getIndex())
      return 0;
    return mValueVector[info.getIndex()];
  }
  const Value* getValue(const Info& info) const
  {
    if (mValueVector.size() <= info.getIndex())
      return 0;
    return mValueVector[info.getIndex()];
  }

  // FIXME: can I avoid that in this class??,
  void setValue(const Info& info, const LeafContext& leafContext)
  {
    if (mValueVector.size() <= info.getIndex())
      mValueVector.resize(info.getIndex()+1);
    mValueVector[info.getIndex()] = info.newStateValue(leafContext);
  }

  void setValue(const Info& info, Value* value)
  {
    if (mValueVector.size() <= info.getIndex())
      mValueVector.resize(info.getIndex()+1);
    mValueVector[info.getIndex()] = value;
  }

  template<typename T>
  typename T::reference operator[](const T& info)
  { return info.getValueReference(getValue(info)); }
  template<typename T>
  typename T::const_reference operator[](const T& info) const
  { return info.getValueReference(getValue(info)); }

protected:
  typedef std::vector<SharedPtr<Value> > ValueVector;
  ValueVector mValueVector;
};

} // namespace OpenFDM

#endif
