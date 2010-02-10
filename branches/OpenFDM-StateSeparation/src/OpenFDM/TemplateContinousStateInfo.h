/* -*-c++-*- OpenFDM - Copyright (C) 2007-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_TemplateContinousStateInfo_H
#define OpenFDM_TemplateContinousStateInfo_H

#include "ContinousStateInfo.h"
#include "ContinousStateValue.h"
#include "TemplateStateInfo.h"

namespace OpenFDM {

template<typename T>
class TemplateContinousStateInfo : public ContinousStateInfo {
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

  reference getValue(TemplateValueVector<ContinousStateInfo,ContinousStateValue>& valueVector) const
  { return getValueReference(valueVector.getValue(*this)); }
  const_reference getValue(const TemplateValueVector<ContinousStateInfo,ContinousStateValue>& valueVector) const
  { return getValueReference(valueVector.getValue(*this)); }

protected:
  struct Value : public ContinousStateValue {
    value_type mValue;

    void resize(const Size& size)
    { mValue.resize(size(0), size(1)); }
    
    const Matrix& getMatrix() const
    { return mValue; }
    Matrix& getMatrix()
    { return mValue; }
    void setMatrix(const Matrix& matrix)
    { OpenFDMAssert(size(matrix) == size(mValue)); mValue = matrix; }
    
    virtual void setValue(const StateStream& stateStream)
    { stateStream.readSubState(mValue); }
    virtual void getValue(StateStream& stateStream) const
    { stateStream.writeSubState(mValue); }
    virtual LinAlg::size_type getNumStates() const
    { return rows(mValue)*cols(mValue); }
  };

  virtual Value* newStateValueImplementation(const LeafContext&) const
  { return new Value; }
};

} // namespace OpenFDM

#endif
