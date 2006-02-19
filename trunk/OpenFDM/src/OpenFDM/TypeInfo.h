/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_TypeInfo_H
#define OpenFDM_TypeInfo_H

#include <string>
#include <vector>
#include <map>

#include "Referenced.h"
#include "SharedPtr.h"
#include "Mutex.h"
#include "ScopeLock.h"
#include "Variant.h"

namespace OpenFDM {

class TypeInfo {
public:
  TypeInfo(const char* name) : mName(name) {}

  const char* const getName(void) const
  { return mName; }

//   void getPropertyList();

protected:
  virtual ~TypeInfo(void) {}

  /// Guard for the stl containers in the TypeInfo
  /// This *should* not be required but the standard does not provide
  /// guarantees about that. Access through the reflection framework
  /// should not happen on the fast path, so do that here ...
  mutable Mutex mMutex;

private:
  TypeInfo(const TypeInfo&);
  TypeInfo& operator=(const TypeInfo&);

  /// The reflected types name
  const char* mName;

  /// The list of baseclasses
  std::vector<const TypeInfo*> mBase;
};

class PropertyInfo {
public:
  enum { NotSerialized = 0, Serialized = 1 };

  PropertyInfo(const char* name, bool isSerialized) :
    mName(name), mIsSerialized(isSerialized)
  {}

  const char* const getName(void) const
  { return mName; }

  bool isSerialized(void) const
  { mIsSerialized; }

  // const TypeInfo& getTypeInfo(void) const;

private:
  const char* mName;
  bool mIsSerialized : 1;
};

template<typename O>
class PropertyAccessor : public Referenced {
public:
  PropertyAccessor(const char* name, bool isSerialized) :
    mPropertyInfo(name, isSerialized)
  {}
  virtual ~PropertyAccessor(void)
  {}

  virtual bool getPropertyValue(const O* object, Variant& value) const = 0;
  virtual bool setPropertyValue(O*, const Variant&) const = 0;

  const PropertyInfo& getPropertyInfo(void) const
  { return mPropertyInfo; }

private:
  PropertyInfo mPropertyInfo;
};

template<typename O, typename T>
class TypedPropertyAccessor : public PropertyAccessor<O> {
public:
  typedef const T& (O::*Getter) () const;
  typedef void (O::*Setter) (const T&);

  TypedPropertyAccessor(const char* name, bool isSerialized,
                        Getter getter, Setter setter) :
    PropertyAccessor<O>(name, isSerialized),
    mGetter(getter),
    mSetter(setter)
  { }

  virtual bool getPropertyValue(const O* object, Variant& value) const
  {
    if (object && mGetter) {
      value = Variant((object->*mGetter)());
      return true;
    } else
      return false;
  }
  virtual bool setPropertyValue(O* object, const Variant& value) const
  {
    if (object && mSetter) {
      /// FIXME, can we do that without copying???
      T tmp;
      variant_copy(value, tmp);
      (object->*mSetter)(tmp);
      return true;
    } else
      return false;
  }
private:
  Getter mGetter;
  Setter mSetter;
};

template<typename O, typename T>
class TypedPropertyAccessor2 : public PropertyAccessor<O> {
public:
  typedef T (O::*Getter) () const;
  typedef void (O::*Setter) (T);

  TypedPropertyAccessor2(const char* name, bool isSerialized,
                         Getter getter, Setter setter) :
    PropertyAccessor<O>(name, isSerialized),
    mGetter(getter),
    mSetter(setter)
  { }

  virtual bool getPropertyValue(const O* object, Variant& value) const
  {
    if (object && mGetter) {
      value = Variant((object->*mGetter)());
      return true;
    } else
      return false;
  }
  virtual bool setPropertyValue(O* object, const Variant& value) const
  {
    if (object && mSetter) {
      /// FIXME, can we do that without copying???
      T tmp;
      variant_copy(value, tmp);
      (object->*mSetter)(tmp);
      return true;
    } else
      return false;
  }
private:
  Getter mGetter;
  Setter mSetter;
};

template<typename O>
class TypeInfoTemplate : public TypeInfo {
public:
  /// Use that in the getter and setter macros, very nice ...
  /// Thanks to Marco Jez from OpenSceneGraph
  typedef O ReflectedType;

  TypeInfoTemplate(const char* name) : TypeInfo(name) {}
  
  bool
  getPropertyValue(const O* obj, const std::string& name, Variant& value) const
  {
    ScopeLock scopeLock(mMutex);
    
    typename std::vector<SharedPtr<PropertyAccessor<O> > >::const_iterator it;
    it = mProperyList.begin();
    while (it != mProperyList.end()) {
      PropertyInfo propInfo = (*it)->getPropertyInfo();
      if (name == propInfo.getName())
        return (*it)->getPropertyValue(obj, value);
      ++it;
    }
    return false;
  }

  bool
  setPropertyValue(O* obj, const std::string& name, const Variant& value) const
  {
    ScopeLock scopeLock(mMutex);

    typename std::vector<SharedPtr<PropertyAccessor<O> > >::const_iterator it;
    it = mProperyList.begin();
    while (it != mProperyList.end()) {
      PropertyInfo propInfo = (*it)->getPropertyInfo();
      if (name == propInfo.getName())
        return (*it)->setPropertyValue(obj, value);
      ++it;
    }
    return false;
  }

  void getPropertyInfoList(std::vector<PropertyInfo>& props) const
  {
    ScopeLock scopeLock(mMutex);

    typename std::vector<SharedPtr<PropertyAccessor<O> > >::const_iterator it;
    it = mProperyList.begin();
    while (it != mProperyList.end()) {
      props.push_back((*it)->getPropertyInfo());
      ++it;
    }
  }

protected:
  template<typename T>
  void addProperty(const char* name, bool serializable,
                   const T& (O::*getter) () const, 
                   void (O::*setter) (const T&))
  {
    ScopeLock scopeLock(mMutex);
    mProperyList.push_back(new TypedPropertyAccessor<O, T>(name, serializable,
                                                           getter, setter));
  }

  template<typename T>
  void addProperty(const char* name, bool serializable,
                   T (O::*getter) () const, 
                   void (O::*setter) (T))
  {
    ScopeLock scopeLock(mMutex);
    mProperyList.push_back(new TypedPropertyAccessor2<O, T>(name, serializable,
                                                            getter, setter));
  }

  template<typename T>
  void addProperty(const char* name, const T& (O::*getter) () const)
  {
    ScopeLock scopeLock(mMutex);
    mProperyList.push_back(new TypedPropertyAccessor<O, T>(name, false,
                                                           getter, 0));
  }
  
  template<typename T>
  void addProperty(const char* name, T (O::*getter) () const)
  {
    ScopeLock scopeLock(mMutex);
    mProperyList.push_back(new TypedPropertyAccessor2<O, T>(name, false,
                                                            getter, 0));
  }
  
private:
  TypeInfoTemplate(const TypeInfoTemplate&);
  TypeInfoTemplate& operator=(const TypeInfoTemplate&);
  
  std::vector<SharedPtr<PropertyAccessor<O> > > mProperyList;
};

} // namespace OpenFDM

#endif
