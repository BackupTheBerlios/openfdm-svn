/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Expression_H
#define OpenFDM_Expression_H

#include <list>
#include <vector>
#include <map>

#include "Assert.h"
#include "LogStream.h"
#include "Types.h"
#include "Units.h"
#include "Object.h"
#include "Property.h"
#include "Table.h"

namespace OpenFDM {

template<typename T>
class ConstExpressionPropertyImpl : public PropertyImpl<T> {
public:
  ConstExpressionPropertyImpl(const T& value) : mValue(value) {}
  virtual void setValue(const T& value) { mValue = value; }
  virtual const Object* getObject(void) const { return 0; }
  virtual Object* getObject(void) { return 0; }
  virtual bool isValid(void) const { return true; }
  virtual T getValue(void) const { return mValue; }
private:
  T mValue;
};

class RealCastExpressionPropertyImpl : public PropertyImpl<real_type> {
public:
  RealCastExpressionPropertyImpl(const Property& prop) : mProperty(prop) {}
  virtual void setValue(const real_type& value) { mProperty.setValue(value); }
  virtual const Object* getObject(void) const { return 0; }
  virtual Object* getObject(void) { return 0; }
  virtual bool isValid(void) const { return mProperty.isValid(); }
  virtual real_type getValue(void) const { return mProperty.getValue().toReal(); }
private:
  mutable Property mProperty;
};

template<typename T>
class ExpressionPropertyImpl : public PropertyImpl<T> {
public:
  virtual void setValue(const T&) { }
  virtual const Object* getObject(void) const { return 0; }
  virtual Object* getObject(void) { return 0; }
};

template<typename T>
class UnaryExpressionImpl :
    public ExpressionPropertyImpl<T> {
public:
  UnaryExpressionImpl(void) {}
  virtual ~UnaryExpressionImpl(void) {}

  virtual bool isValid(void) const { return mInput.isValid(); }

  void setInputProperty(const Property& prop)
  {
    Property tmpHack(prop);
    TypedProperty<T> tProp = tmpHack.toTypedProperty<T>();
    if (tProp.isValid())
      setInputProperty(tProp);
    else {
      Property prop2(new RealCastExpressionPropertyImpl(tmpHack));
      setInputProperty(prop2.toTypedProperty<T>());
    }
  }
  void setInputProperty(const TypedProperty<T>& prop)
  { mInput = prop; }

protected:
  mutable TypedProperty<T> mInput;
};

/// Implementations of various similar functions.
class AbsExpressionImpl :
    public UnaryExpressionImpl<real_type> {
private:
  virtual real_type getValue(void) const
  { return fabs(mInput.getValue()); }
};

class MinusExpressionImpl :
    public UnaryExpressionImpl<real_type> {
private:
  virtual real_type getValue(void) const
  { return -mInput.getValue(); }
};

class UnitToSiExpressionImpl :
    public UnaryExpressionImpl<real_type> {
public:
  UnitToSiExpressionImpl(Unit unit) : mUnit(unit) {}
private:
  virtual real_type getValue(void) const
  { return convertFrom(mUnit, mInput.getValue()); }
  Unit mUnit;
};

class SiToUnitExpressionImpl :
    public UnaryExpressionImpl<real_type> {
public:
  SiToUnitExpressionImpl(Unit unit) : mUnit(unit) {}
private:
  virtual real_type getValue(void) const
  { return convertTo(mUnit, mInput.getValue()); }
  Unit mUnit;
};

template<typename T>
class MultiExpressionImpl :
    public ExpressionPropertyImpl<T> {
public:
  MultiExpressionImpl(void) {}
  virtual ~MultiExpressionImpl(void) {}

  virtual bool isValid(void) const
  {
    for (unsigned i = 0; i < mInputs.size(); ++i)
      if (!mInputs[i].isValid())
        return false;
    return true;
  }
  
  void setNumInputs(unsigned numInputs)
  { mInputs.resize(numInputs); }
  unsigned getNumInputs(void) const
  { return mInputs.size(); }

  void setInputProperty(unsigned idx, const Property& prop)
  {
    Property tmpHack(prop);
    TypedProperty<T> tProp = tmpHack.toTypedProperty<T>();
    if (tProp.isValid())
      setInputProperty(idx, tProp);
    else {
      Property prop2(new RealCastExpressionPropertyImpl(tmpHack));
      setInputProperty(idx, prop2.toTypedProperty<T>());
    }
  }
  void setInputProperty(unsigned idx, const TypedProperty<T>& prop)
  { mInputs[idx%mInputs.size()] = prop; }

  void addInputProperty(const Property& prop)
  {
    Property tmpHack(prop);
    TypedProperty<T> tProp = tmpHack.toTypedProperty<T>();
    if (tProp.isValid())
      addInputProperty(tProp);
    else {
      Property prop2(new RealCastExpressionPropertyImpl(tmpHack));
      addInputProperty(prop2.toTypedProperty<T>());
    }
  }
  void addInputProperty(const TypedProperty<T>& prop)
  { mInputs.push_back(prop); }

protected:
  mutable std::vector<TypedProperty<T> > mInputs;
};

class SumExpressionImpl :
    public MultiExpressionImpl<real_type> {
public:
  virtual real_type getValue(void) const
  {
    real_type sum = 0;
    for (unsigned i = 0; i < mInputs.size(); ++i)
      sum += mInputs[i].getValue();
    return sum;
  }
};

class ProductExpressionImpl :
    public MultiExpressionImpl<real_type> {
public:
  virtual real_type getValue(void) const
  {
    real_type prod = 1;
    for (unsigned i = 0; i < mInputs.size(); ++i)
      prod *= mInputs[i].getValue();
    return prod;
  }
};

template<unsigned numDims>
class TableExpressionImpl :
    public ExpressionPropertyImpl<real_type> {
public:
  TableExpressionImpl(void) {}
  virtual ~TableExpressionImpl(void) {}

  void setTable(const TableData<numDims>& table)
  { mTableData = table; }

  const TableData<numDims>& getTable(void) const 
  { return mTableData; }
  TableData<numDims>& getTable(void)
  { return mTableData; }

  virtual bool isValid(void) const
  {
    for (unsigned i = 0; i < numDims; ++i)
      if (!mInputs[i].isValid())
        return false;
    return true;
  }

  void setInputProperty(unsigned idx, const Property& prop)
  {
    Property tmpHack(prop);
    TypedProperty<real_type> tProp = tmpHack.toTypedProperty<real_type>();
    if (tProp.isValid())
      setInputProperty(idx, tProp);
    else {
      Property prop2(new RealCastExpressionPropertyImpl(tmpHack));
      setInputProperty(idx, prop2.toTypedProperty<real_type>());
    }
  }
  void setInputProperty(unsigned idx, const TypedProperty<real_type>& prop)
  { mInputs[idx%numDims] = prop; }

  void setTableLookup(unsigned idx, const TableLookup& tl)
  {
    OpenFDMAssert(idx < numDims);
    typename TableData<numDims>::SizeVector sz = mTableData.size();
    OpenFDMAssert(sz(idx+1) == tl.size());
    mTableLookups[idx] = tl;
  }

  virtual real_type getValue(void) const
  {
    typename TableData<numDims>::InterpVector interpVec;
    for (unsigned i = 0; i < numDims; ++i) {
      real_type iv = mInputs[i].getValue();
      interpVec(i+1) = mTableLookups[i].lookup(iv);
    }
    return mTableData.interpolate(interpVec);
  }

protected:
  mutable TypedProperty<real_type> mInputs[numDims]; /*FIXME*/
  TableLookup mTableLookups[numDims];
  TableData<numDims> mTableData;
};

} // namespace OpenFDM

#endif
