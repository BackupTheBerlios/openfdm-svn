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
class shared_ptr;
template<typename T>
class managed_ptr;

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

  shared_ptr<Object> mUserData;

  mutable std::list<Object**> _ptrList;

  template<typename T>
  friend class shared_ptr;
  template<typename T>
  friend class managed_ptr;
};


/// FIXME: remove the direct accessors, only copy to a shared_ptr
/// Where you can access then, may be similar to the std::tr2::weak_ptr::lock()
/// function. That is to avoid deletioin of a currently used object
template<typename T>
class managed_ptr {
public:
  managed_ptr(void) : _ptr(0)
  {}
  managed_ptr(T* ptr) : _ptr(ptr)
  { reg(); }
  managed_ptr(const managed_ptr& p) : _ptr(p._ptr)
  { reg(); }
  template<typename U>
  managed_ptr(const shared_ptr<U>& p) : _ptr(p._ptr)
  { reg(); }
  ~managed_ptr(void)
  { dereg(); }
  
  template<typename U>
  managed_ptr& operator=(const shared_ptr<U>& p)
  { assign(p._ptr); return *this; }
  template<typename U>
  managed_ptr& operator=(U* p)
  { assign(p); return *this; }
  managed_ptr& operator=(const managed_ptr& p)
  { assign(p._ptr); return *this; }
  template<typename U>
  managed_ptr& operator=(const managed_ptr<U>& p)
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
  friend class shared_ptr;
  template<typename U>
  friend class managed_ptr;
};

} // namespace OpenFDM

#endif
