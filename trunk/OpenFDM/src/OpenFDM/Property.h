/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Property_H
#define OpenFDM_Property_H

#include "Assert.h"
#include "Referenced.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Plane.h"
#include "TableData.h"
#include "Variant.h"

namespace OpenFDM {

class Object;

class PropertyImpl :
    public Referenced {
public:
  PropertyImpl(bool isStored = false) : mIsStored(isStored) {}
  virtual ~PropertyImpl(void);

  virtual Variant getValue(void) = 0;
  virtual void setValue(const Variant& value) = 0;

  virtual bool isValid(void) const = 0;

  void setStoredProperty(bool isStored)
  { mIsStored = isStored; }
  bool isStoredProperty(void) const
  { return mIsStored; }

private:
  bool mIsStored;

  /// Properties are not assignable.
  PropertyImpl(const PropertyImpl&);
  PropertyImpl& operator=(const PropertyImpl&);
};

template<typename O, typename T>
class ObjectPropertyImpl : public PropertyImpl {
public:
  typedef const T& (O::*GetRefMethod)(void) const;
  typedef T (O::*GetMethod)(void) const;
  typedef void (O::*SetMethod)(const T&);

  ObjectPropertyImpl(O* object, GetMethod getMethod) :
    mObject(object),
    mGetMethod(getMethod), mGetRefMethod(0), mSetMethod(0)
  {}
  ObjectPropertyImpl(O* object, GetRefMethod getRefMethod) :
    mObject(object),
    mGetMethod(0), mGetRefMethod(getRefMethod), mSetMethod(0)
  {}
  ObjectPropertyImpl(O* object, SetMethod setMethod) :
    mObject(object),
    mGetMethod(0), mGetRefMethod(0), mSetMethod(setMethod)
  {}
  ObjectPropertyImpl(O* object, GetMethod getMethod, SetMethod setMethod) :
    mObject(object),
    mGetMethod(getMethod), mGetRefMethod(0), mSetMethod(setMethod)
  {}
  ObjectPropertyImpl(O* object, GetRefMethod getRefMethod, SetMethod setMethod) :
    mObject(object),
    mGetMethod(0), mGetRefMethod(getRefMethod), mSetMethod(setMethod)
  {}

  /// Retrieve property from the given object.
  virtual Variant getValue(void)
  {
    const O* object = mObject;
    OpenFDMAssert(object);
    if (mGetRefMethod && object)
      return Variant((object->*mGetRefMethod)());
    else if (mGetMethod && object)
      return Variant((object->*mGetMethod)());
    else
      return Variant();
  }

  /// Set property to the given object.
  virtual void setValue(const Variant& value)
  {
    T typedValue;
    variant_copy(value, typedValue);
    O* object = mObject;
    OpenFDMAssert(object);
    if (mSetMethod && object)
      (object->*mSetMethod)(typedValue);
  }

  virtual bool isValid(void) const { return mObject; }

private:
  WeakPtr<O> mObject;
  GetMethod mGetMethod;
  GetRefMethod mGetRefMethod;
  SetMethod mSetMethod;
};

class Property {
public:
  Property(void) {}
  Property(PropertyImpl* propertyImpl) :
    mPropertyImpl(propertyImpl)
  {}
  template<typename O, typename T>
  Property(O* object, T (O::*getter) () const) :
    mPropertyImpl(new ObjectPropertyImpl<O,T>(object, getter))
  {}
  template<typename O, typename T>
  Property(O* object, const T& (O::*getter) () const) :
    mPropertyImpl(new ObjectPropertyImpl<O,T>(object, getter))
  {}
  template<typename O, typename T>
  Property(O* object, void (O::*setter) (const T&)) :
    mPropertyImpl(new ObjectPropertyImpl<O,T>(object, setter))
  {}
  template<typename O, typename T>
  Property(O* object, T (O::*getter) () const, void (O::*setter) (const T&)) :
    mPropertyImpl(new ObjectPropertyImpl<O,T>(object, getter, setter))
  {}
  template<typename O, typename T>
  Property(O* object, const T& (O::*getter) () const, void (O::*setter) (const T&)) :
    mPropertyImpl(new ObjectPropertyImpl<O,T>(object, getter, setter))
  {}

  /// Can always do that via Variants.
  /// Take care that this is costly ...
  Variant getValue(void)
  {
    if (mPropertyImpl)
      return mPropertyImpl->getValue();
    else
      return Variant();
  }
  void setValue(const Variant& value)
  {
    if (mPropertyImpl)
      mPropertyImpl->setValue(value);
  }

  /// Returns if the property contains a valid property reference and
  /// references an existing object.
  bool isValid(void) const
  { return mPropertyImpl && mPropertyImpl->isValid(); }

  void setStoredProperty(bool isStored)
  { if (mPropertyImpl) mPropertyImpl->setStoredProperty(isStored); }
  bool isStoredProperty(void) const
  { return mPropertyImpl && mPropertyImpl->isStoredProperty(); }

private:
  SharedPtr<PropertyImpl> mPropertyImpl;
};

} // namespace OpenFDM

#endif
