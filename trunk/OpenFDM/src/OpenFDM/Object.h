/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Object_H
#define OpenFDM_Object_H

#include <list>
#include <map>
#include "Referenced.h"
#include "Variant.h"
#include "Property.h"

namespace OpenFDM {

template<typename T>
class SharedPtr;
template<typename T>
class WeakPtr;

class TypeInfo;

class Object
  : public Referenced {
public:
  Object(const std::string& name = std::string());

  /// Returns the systems name.
  const std::string& getName(void) const
  { return mName; }
  void setName(const std::string& name)
  { mName = name; }

  /// Return the typeinfo for that Object.
  virtual const TypeInfo* const getTypeInfo(void) const;
  
  Property getProperty(const std::string& name);
  std::list<std::string> listProperties(void) const;

  /// Returns the objects property named name
  Variant getPropertyValue(const std::string& name) const;
  /// Set an objects property named name to the given value
  void setPropertyValue(const std::string& name, const Variant& value);

  /// Returns the objects attached user data
  Object* getUserData(void)
  { return mUserData; }
  /// Returns the objects attached const user data
  const Object* getUserData(void) const
  { return mUserData; }
  /// Sets the objects user data
  void setUserData(Object* userData)
  { mUserData = userData; }

protected:
  /// Defines the map carrying the properties for this object.
  typedef std::map<std::string,Property> PropertyMap;

  // Only destructable by the reference counted pointers.
  virtual ~Object(void);

  void addProperty(const std::string& name, const Property& property);
  void removeProperty(const std::string& name);

private:
  /// Such objects can not be copied.
  Object(const Object&);
  const Object& operator=(const Object&);

  /// Register and deregister a managed reference to this object.
  void reg(Object** mp);
  void dereg(Object** mp);

  /// The objects name
  std::string mName;

  /// The map of all properties of this object.
  PropertyMap mProperties;

  SharedPtr<Object> mUserData;

  mutable std::list<Object**> _ptrList;

  template<typename T>
  friend class SharedPtr;
  template<typename T>
  friend class WeakPtr;
};


/// FIXME: remove the direct accessors, only copy to a SharedPtr
/// where you can access then, may be similar to the std::tr2::weak_ptr::lock()
/// function. That is to avoid deletion of a currently used object
/// FIXME make const correct ...
template<typename T>
class WeakPtr {
public:
  WeakPtr(void) : _ptr(0)
  {}
  WeakPtr(T* ptr) : _ptr(ptr)
  { reg(); }
  WeakPtr(const WeakPtr& p) : _ptr(p._ptr)
  { reg(); }
  template<typename U>
  WeakPtr(const SharedPtr<U>& p) : _ptr(p._ptr)
  { reg(); }
  ~WeakPtr(void)
  { dereg(); }
  
  template<typename U>
  WeakPtr& operator=(const SharedPtr<U>& p)
  { assign(p._ptr); return *this; }
  template<typename U>
  WeakPtr& operator=(U* p)
  { assign(p); return *this; }
  WeakPtr& operator=(const WeakPtr& p)
  { assign(p._ptr); return *this; }
  template<typename U>
  WeakPtr& operator=(const WeakPtr<U>& p)
  { assign(p._ptr); return *this; }

  T* operator->(void)
  { return reinterpret_cast<T*>(_ptr); }
  const T* operator->(void) const
  { return reinterpret_cast<const T*>(_ptr); }
  T& operator*(void)
  { return *reinterpret_cast<T*>(_ptr); }
  const T& operator*(void) const
  { return *reinterpret_cast<const T*>(_ptr); }

  operator T*(void)
  { return reinterpret_cast<T*>(_ptr); }
  operator const T*(void) const
  { return reinterpret_cast<const T*>(_ptr); }

private:
  template<typename U>
  void assign(U* p)
  { dereg(); _ptr = p; reg(); }
  
  void reg(void)
  { if (_ptr) _ptr->reg(&_ptr); }

  void dereg(void)
  { if (_ptr) _ptr->dereg(&_ptr); }

  // The reference itself.
  Object* _ptr;

  template<typename U>
  friend class SharedPtr;
  template<typename U>
  friend class WeakPtr;
};

} // namespace OpenFDM

#endif
