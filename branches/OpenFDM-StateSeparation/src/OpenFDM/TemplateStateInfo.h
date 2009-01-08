/* -*-c++-*- OpenFDM - Copyright (C) 2007-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_TemplateStateInfo_H
#define OpenFDM_TemplateStateInfo_H

#include "TemplateValueVector.h"

namespace OpenFDM {

template<typename T, typename I, typename V>
class TemplateStateInfo : public I {
public:
  typedef T value_type;

  typedef value_type& reference;
  typedef const value_type& const_reference;

//   reference getValueReference(V* value) const
  reference getValueReference(StateValue* value) const
  {
    OpenFDMAssert(value);
    OpenFDMAssert(dynamic_cast<Value*>(value));
    return static_cast<Value*>(value)->mValue;
  }
//   const_reference getValueReference(const V* value) const
  const_reference getValueReference(const StateValue* value) const
  {
    OpenFDMAssert(value);
    OpenFDMAssert(dynamic_cast<const Value*>(value));
    return static_cast<const Value*>(value)->mValue;
  }

  reference getValue(TemplateValueVector<I,V>& valueVector) const
  { return getValueReference(valueVector.getValue(*this)); }
  const_reference getValue(const TemplateValueVector<I,V>& valueVector) const
  { return getValueReference(valueVector.getValue(*this)); }

protected:
  struct Value : public V {
    value_type mValue;
  };

  virtual Value* newStateValueImplementation(const LeafContext&) const
  { return new Value; }
};

} // namespace OpenFDM

#endif
