/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
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

/// Macros for the reflection stuff
#define OPENFDM_OBJECT(classname, baseclassname)                \
public: virtual const char* getTypeName(void) const

#define BEGIN_OPENFDM_OBJECT_DEF(classname, baseclassname)      \
const char*                                                     \
classname::getTypeName(void) const                              \
{                                                               \
  return #classname ;                                           \
}                                                               \

#define END_OPENFDM_OBJECT_DEF


/// The OpenFDM object base class.
class Object : public Referenced {
public:
  Object(const std::string& name = std::string());
  virtual ~Object(void);

  /// Returns the systems name.
  const std::string& getName(void) const
  { return mName; }
  void setName(const std::string& name)
  { mName = name; }

  /// Return the typeinfo for that Object.
  virtual const char* getTypeName(void) const;
  
  std::list<std::string> listProperties(void) const;

  /// Returns the objects property named name
  Variant getPropertyValue(const std::string& name) const;
  /// Set an objects property named name to the given value
  void setPropertyValue(const std::string& name, const Variant& value);

  /// Returns true if the property must be stored to reflect this given object
  bool isStoredProperty(const std::string& name) const;

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

  void addProperty(const std::string& name, const Property& property);
  void addStoredProperty(const std::string& name, const Property& property);

private:
  /// Such objects can not be copied.
  Object(const Object&);
  const Object& operator=(const Object&);

  /// The objects name
  std::string mName;

  /// The map of all properties of this object.
  PropertyMap mProperties;

  /// Userdata ...
  SharedPtr<Object> mUserData;

  /// Support for weak references, not increasing the reference count
  /// that is done through that small helper class which holds an uncounted
  /// reference which is zeroed out on destruction of the current object
  struct WeakData : public Referenced {
    WeakData(Object* o) : object(o) {}
    Object* object;
  private:
    WeakData(void);
    WeakData(const WeakData&);
    WeakData& operator=(const WeakData&);
  };
  SharedPtr<WeakData> mWeakDataPtr;

  template<typename T>
  friend class SharedPtr;
  template<typename T>
  friend class WeakPtr;
};


/// FIXME: remove the direct accessors, only copy to a SharedPtr
/// where you can access then, may be similar to the std::tr2::weak_ptr::lock()
/// function. That is to avoid deletion of a currently used object
template<typename T>
class WeakPtr {
public:
  WeakPtr(void)
  {}
  WeakPtr(T* ptr)
  { assign(ptr); }
  WeakPtr(const WeakPtr& p) : mWeakDataPtr(p.mWeakDataPtr)
  { }
  template<typename U>
  WeakPtr(const SharedPtr<U>& p)
  { assign(p.ptr()); }
  ~WeakPtr(void)
  { }
  
  template<typename U>
  WeakPtr& operator=(const SharedPtr<U>& p)
  { assign(p.ptr()); return *this; }
  template<typename U>
  WeakPtr& operator=(U* p)
  { assign(p); return *this; }
  WeakPtr& operator=(const WeakPtr& p)
  { mWeakDataPtr = p.mWeakDataPtr; return *this; }

  T* operator->(void) const
  { return ptr(); }

  T& operator*(void) const
  { return *ptr(); }

  operator T*(void) const
  { return ptr(); }

private:
  Object* objectPtr(void) const
  { return mWeakDataPtr ? mWeakDataPtr->object : 0; }
  T* ptr(void) const
  { return reinterpret_cast<T*>(objectPtr()); }
  void assign(T* p)
  {
    if (p)
      mWeakDataPtr = p->mWeakDataPtr;
    else
      mWeakDataPtr = 0;
  }
  
  // The indirect reference itself.
  SharedPtr<Object::WeakData> mWeakDataPtr;

  template<typename U>
  friend class SharedPtr;
  template<typename U>
  friend class WeakPtr;
};

} // namespace OpenFDM

#endif
