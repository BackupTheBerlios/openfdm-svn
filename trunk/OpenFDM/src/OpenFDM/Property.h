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
#include "Variant.h"

namespace OpenFDM {

class Object;

template<typename T>
class PropertyImpl;

typedef PropertyImpl<int> IntegerPropertyImpl;
typedef PropertyImpl<unsigned> UnsignedPropertyImpl;
typedef PropertyImpl<real_type> RealPropertyImpl;
typedef PropertyImpl<Vector2> Vector2PropertyImpl;
typedef PropertyImpl<Vector3> Vector3PropertyImpl;
typedef PropertyImpl<Quaternion> QuaternionPropertyImpl;
typedef PropertyImpl<Plane> PlanePropertyImpl;
typedef PropertyImpl<Vector6> Vector6PropertyImpl;
typedef PropertyImpl<Matrix> MatrixPropertyImpl;
typedef PropertyImpl<std::string> StringPropertyImpl;

class UntypedPropertyImpl :
    public Referenced {
public:
  UntypedPropertyImpl(void) {}
  virtual ~UntypedPropertyImpl(void);

  /// Can always do that via Variants.
  /// Take care that this is costly ...
  Variant getValue(void) /*const FIXME*/;
  void setValue(const Variant& value);

  virtual bool isValid(void) const = 0;
  virtual const Object* getObject(void) const = 0;
  virtual Object* getObject(void) = 0;

  virtual IntegerPropertyImpl* toIntegerPropertyImpl(void);
  virtual UnsignedPropertyImpl* toUnsignedPropertyImpl(void);
  virtual RealPropertyImpl* toRealPropertyImpl(void);
  virtual Vector2PropertyImpl* toVector2PropertyImpl(void);
  virtual Vector3PropertyImpl* toVector3PropertyImpl(void);
  virtual QuaternionPropertyImpl* toQuaternionPropertyImpl(void);
  virtual PlanePropertyImpl* toPlanePropertyImpl(void);
  virtual Vector6PropertyImpl* toVector6PropertyImpl(void);
  virtual MatrixPropertyImpl* toMatrixPropertyImpl(void);
  virtual StringPropertyImpl* toStringPropertyImpl(void);

private:
  /// Properties are not assignable.
  UntypedPropertyImpl(const UntypedPropertyImpl&);
  UntypedPropertyImpl& operator=(const UntypedPropertyImpl&);
};

template<typename T>
class PropertyImplInterface : public UntypedPropertyImpl {
public:
  /// Retrieve property from the given object.
  virtual T getValue(void) const = 0;
  /// Set property of the given object.
  virtual void setValue(const T&) = 0;
};

template<typename T>
class PropertyImpl : public PropertyImplInterface<T> {};
template<>
class PropertyImpl<int> : public PropertyImplInterface<int> {
  virtual IntegerPropertyImpl* toIntegerPropertyImpl(void) { return this; }
};
template<>
class PropertyImpl<unsigned> : public PropertyImplInterface<unsigned> {
  virtual UnsignedPropertyImpl* toUnsignedPropertyImpl(void) { return this; }
};
template<>
class PropertyImpl<real_type> : public PropertyImplInterface<real_type> {
  virtual RealPropertyImpl* toRealPropertyImpl(void) { return this; }
};
template<>
class PropertyImpl<Vector2> : public PropertyImplInterface<Vector2> {
  virtual Vector2PropertyImpl* toVector2PropertyImpl(void) { return this; }
};
template<>
class PropertyImpl<Vector3> : public PropertyImplInterface<Vector3> {
  virtual Vector3PropertyImpl* toVector3PropertyImpl(void) { return this; }
};
template<>
class PropertyImpl<Quaternion> : public PropertyImplInterface<Quaternion> {
  virtual QuaternionPropertyImpl* toQuaternionPropertyImpl(void) { return this; }
};
template<>
class PropertyImpl<Plane> : public PropertyImplInterface<Plane> {
  virtual PlanePropertyImpl* toPlanePropertyImpl(void) { return this; }
};
template<>
class PropertyImpl<Vector6> : public PropertyImplInterface<Vector6> {
  virtual Vector6PropertyImpl* toVector6PropertyImpl(void) { return this; }
};
template<>
class PropertyImpl<Matrix> : public PropertyImplInterface<Matrix> {
  virtual MatrixPropertyImpl* toMatrixPropertyImpl(void) { return this; }
};
template<>
class PropertyImpl<std::string> : public PropertyImplInterface<std::string> {
  virtual StringPropertyImpl* toStringPropertyImpl(void) { return this; }
};

template<typename O, typename T>
class ObjectPropertyImpl : public PropertyImpl<T> {
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
  virtual T getValue(void) const
  {
    const O* object = mObject;
    OpenFDMAssert(object);
    if (mGetRefMethod && object)
      return (object->*mGetRefMethod)();
    else if (mGetMethod && object)
      return (object->*mGetMethod)();
    else
      return T();
  }

  /// Set property to the given object.
  virtual void setValue(const T& value)
  {
    O* object = mObject;
    OpenFDMAssert(object);
    if (mSetMethod && object)
      (object->*mSetMethod)(value);
  }

  virtual bool isValid(void) const { return mObject; }
  virtual const Object* getObject(void) const { return mObject; }
  virtual Object* getObject(void) { return mObject; }

private:
  WeakPtr<O> mObject;
  GetMethod mGetMethod;
  GetRefMethod mGetRefMethod;
  SetMethod mSetMethod;
};

template<typename T>
class TypedProperty {
public:
  typedef PropertyImpl<T> implementation_t;

  TypedProperty(implementation_t* propertyImpl = 0) :
    mPropertyImpl(propertyImpl)
  {}
  template<typename O>
  TypedProperty(O* object, T (O::*getter) () const) :
    mPropertyImpl(new ObjectPropertyImpl<O,T>(object, getter))
  {}
  template<typename O>
  TypedProperty(O* object, const T& (O::*getter) () const) :
    mPropertyImpl(new ObjectPropertyImpl<O,T>(object, getter))
  {}
  template<typename O>
  TypedProperty(O* object, void (O::*setter) (const T&)) :
    mPropertyImpl(new ObjectPropertyImpl<O,T>(object, setter))
  {}
  template<typename O>
  TypedProperty(O* object, T (O::*getter) () const, void (O::*setter) (const T&)) :
    mPropertyImpl(new ObjectPropertyImpl<O,T>(object, getter, setter))
  {}
  template<typename O>
  TypedProperty(O* object, const T& (O::*getter) () const, void (O::*setter) (const T&)) :
    mPropertyImpl(new ObjectPropertyImpl<O,T>(object, getter, setter))
  {}

  /// Returns if the property contains a valid property reference and
  /// references an existing object.
  bool isValid(void) const
  { return mPropertyImpl && mPropertyImpl->isValid(); }

  /// Returns the object the property is referencing.
  const Object* getObject(void) const
  { return mPropertyImpl ? mPropertyImpl->getObject() : 0; }
  Object* getObject(void)
  { return mPropertyImpl ? mPropertyImpl->getObject() : 0; }

  T getValue(void) /*const FIXME*/
  {
    if (mPropertyImpl)
      return mPropertyImpl->getValue();
    else
      return T();
  }
  void setValue(const T& value)
  {
    if (mPropertyImpl)
      mPropertyImpl->setValue(value);
  }

private:
  SharedPtr<implementation_t> mPropertyImpl;
};

typedef TypedProperty<int> IntegerProperty;
typedef TypedProperty<unsigned> UnsignedProperty;
typedef TypedProperty<real_type> RealProperty;
typedef TypedProperty<Vector2> Vector2Property;
typedef TypedProperty<Vector3> Vector3Property;
typedef TypedProperty<Quaternion> QuaternionProperty;
typedef TypedProperty<Plane> PlaneProperty;
typedef TypedProperty<Vector6> Vector6Property;
typedef TypedProperty<Matrix> MatrixProperty;
typedef TypedProperty<std::string> StringProperty;

class Property {
public:
  Property(void) {}
  Property(UntypedPropertyImpl* propertyImpl) :
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
  Variant getValue(void) /*const FIXME*/
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

  /// Returns the object the property is referencing.
  const Object* getObject(void) const
  { return mPropertyImpl ? mPropertyImpl->getObject() : 0; }
  Object* getObject(void)
  { return mPropertyImpl ? mPropertyImpl->getObject() : 0; }

  bool isIntegerProperty(void)
  { return mPropertyImpl->toIntegerPropertyImpl(); }
  TypedProperty<int> toIntegerProperty(void)
  { return TypedProperty<int>(mPropertyImpl->toIntegerPropertyImpl()); }

  bool isUnsignedProperty(void)
  { return mPropertyImpl->toUnsignedPropertyImpl(); }
  TypedProperty<unsigned> toUnsignedProperty(void)
  { return TypedProperty<unsigned>(mPropertyImpl->toUnsignedPropertyImpl()); }

  bool isRealProperty(void)
  { return mPropertyImpl->toRealPropertyImpl(); }
  TypedProperty<real_type> toRealProperty(void)
  { return TypedProperty<real_type>(mPropertyImpl->toRealPropertyImpl()); }

  bool isVector2Property(void)
  { return mPropertyImpl->toVector2PropertyImpl(); }
  TypedProperty<Vector2> toVector2Property(void)
  { return TypedProperty<Vector2>(mPropertyImpl->toVector2PropertyImpl()); }

  bool isVector3Property(void)
  { return mPropertyImpl->toVector3PropertyImpl(); }
  TypedProperty<Vector3> toVector3Property(void)
  { return TypedProperty<Vector3>(mPropertyImpl->toVector3PropertyImpl()); }

  bool isQuaternionProperty(void)
  { return mPropertyImpl->toQuaternionPropertyImpl(); }
  TypedProperty<Quaternion> toQuaternionProperty(void)
  { return TypedProperty<Quaternion>(mPropertyImpl->toQuaternionPropertyImpl()); }

  bool isPlaneProperty(void)
  { return mPropertyImpl->toPlanePropertyImpl(); }
  TypedProperty<Plane> toPlaneProperty(void)
  { return TypedProperty<Plane>(mPropertyImpl->toPlanePropertyImpl()); }

  bool isVector6Property(void)
  { return mPropertyImpl->toVector6PropertyImpl(); }
  TypedProperty<Vector6> toVector6Property(void)
  { return TypedProperty<Vector6>(mPropertyImpl->toVector6PropertyImpl()); }

  bool isMatrixProperty(void)
  { return mPropertyImpl->toMatrixPropertyImpl(); }
  TypedProperty<Matrix> toMatrixProperty(void)
  { return TypedProperty<Matrix>(mPropertyImpl->toMatrixPropertyImpl()); }

  bool isStringProperty(void)
  { return mPropertyImpl->toStringPropertyImpl(); }
  TypedProperty<std::string> toStringProperty(void)
  { return TypedProperty<std::string>(mPropertyImpl->toStringPropertyImpl()); }

  template<typename T>
  TypedProperty<T> toTypedProperty(void)
  {
    UntypedPropertyImpl* impl = mPropertyImpl;
    return TypedProperty<T>(dynamic_cast<PropertyImpl<T>*>(impl));
  }

private:
  SharedPtr<UntypedPropertyImpl> mPropertyImpl;
};

} // namespace OpenFDM

#endif
