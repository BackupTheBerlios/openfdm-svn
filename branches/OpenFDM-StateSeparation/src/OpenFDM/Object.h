/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich
 *
 */

#ifndef OpenFDM_Object_H
#define OpenFDM_Object_H

#include <string>
#include <vector>

#include "OpenFDMConfig.h"

#include "SharedPtr.h"
#include "WeakReferenced.h"

namespace OpenFDM {

class Variant;
class PropertyInfo;
class TypeInfo;

/// Macros for the reflection stuff
#define OPENFDM_OBJECT(classname, baseclassname)                              \
private:                                                                      \
  class classname ## TypeInfo;                                                \
  static classname ## TypeInfo sTypeInfo;                                     \
public:                                                                       \
  virtual const TypeInfo& getTypeInfo(void) const;                            \
  virtual bool getPropertyValue(const std::string&, Variant&) const;          \
  virtual bool setPropertyValue(const std::string&, const Variant&);          \
  virtual void getPropertyInfoList(std::vector<PropertyInfo>& props) const


/// Start a reflected object definition
#define BEGIN_OPENFDM_OBJECT_DEF(classname, baseclassname)                    \
class classname :: classname ## TypeInfo :                                    \
  public TypeInfoTemplate<classname> {                                        \
public:                                                                       \
  classname ## TypeInfo(void);                                                \
  ~ classname ## TypeInfo(void);                                              \
private:                                                                      \
  classname ## TypeInfo(const classname ## TypeInfo&);                        \
  classname ## TypeInfo& operator=(const classname  ## TypeInfo&);            \
};                                                                            \
                                                                              \
const TypeInfo&                                                               \
classname :: getTypeInfo(void) const                                          \
{                                                                             \
  return sTypeInfo;                                                           \
}                                                                             \
                                                                              \
bool                                                                          \
classname :: getPropertyValue(const std::string& name, Variant& value) const  \
{                                                                             \
  if (sTypeInfo.getPropertyValue(this, name, value))                          \
    return true;                                                              \
  return baseclassname :: getPropertyValue(name, value);                      \
}                                                                             \
                                                                              \
bool                                                                          \
classname :: setPropertyValue(const std::string& name, const Variant& value)  \
{                                                                             \
  if (sTypeInfo.setPropertyValue(this, name, value))                          \
    return true;                                                              \
  return baseclassname :: setPropertyValue(name, value);                      \
}                                                                             \
                                                                              \
void                                                                          \
classname :: getPropertyInfoList(std::vector<PropertyInfo>& props) const      \
{                                                                             \
  baseclassname :: getPropertyInfoList(props);                                \
  sTypeInfo.getPropertyInfoList(props);                                       \
}                                                                             \
                                                                              \
classname :: classname ## TypeInfo classname :: sTypeInfo;                    \
                                                                              \
classname :: classname ## TypeInfo:: ~ classname ## TypeInfo(void)            \
{}                                                                            \
                                                                              \
classname :: classname ## TypeInfo:: classname ## TypeInfo(void) :            \
  TypeInfoTemplate<classname>(#classname)                                     \
{                                                                             \

/// Used to define an ordinary property which will be used to
/// serialize the model into whatever representation
#define DEF_OPENFDM_PROPERTY(propType, name, serialized)                      \
  addProperty( #name , PropertyInfo:: serialized,                             \
               &ReflectedType::get ## name, &ReflectedType::set ## name );

/// Used to define a read only property which is just available
/// from within the property framework, but is not used to serialize models
#define DEF_OPENFDM_ROPROP(propType, name)                                    \
  addProperty( #name , &ReflectedType::get ## name);

#define END_OPENFDM_OBJECT_DEF                                                \
}

/// The OpenFDM object base class.
class Object : public WeakReferenced {
  OPENFDM_OBJECT(Object, );
public:
  Object(const std::string& name = "Unnamed Object");

  /// Returns the Objects name.
  const std::string& getName(void) const;
  void setName(const std::string& name);

  /// Returns the objects type name
  const char* const getTypeName(void) const;

  /// Returns the objects attached user data
  Object* getUserData(void);
  /// Returns the objects attached const user data
  const Object* getUserData(void) const;
  /// Sets the objects user data
  void setUserData(Object* userData);

  /// overwrites the destruct function in Referenced
  static void destruct(const Object* object);

protected:
  /// Objects must not be put onto the stack
  virtual ~Object(void);

private:
  /// Such objects can not be copied.
  Object(const Object&);
  Object& operator=(const Object&);

  /// The objects name
  std::string mName;

  /// Userdata ...
  SharedPtr<Object> mUserData;
};

} // namespace OpenFDM

#endif
