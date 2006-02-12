/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Object_H
#define OpenFDM_Object_H

#include <list>
#include <map>
#include <string>

#include "SharedPtr.h"
#include "WeakReferenced.h"
#include "Variant.h"
#include "Property.h"

namespace OpenFDM {

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
class Object : public WeakReferenced {
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
};

} // namespace OpenFDM

#endif
