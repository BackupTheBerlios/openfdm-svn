/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "Object.h"

#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

class Object::ObjectTypeInfo : public TypeInfoTemplate<Object> {
public:
  ObjectTypeInfo(void);
  ~ObjectTypeInfo(void);
private:
  ObjectTypeInfo(const ObjectTypeInfo&);
  ObjectTypeInfo& operator=(const ObjectTypeInfo&);
};

Object::ObjectTypeInfo::ObjectTypeInfo(void) :
  TypeInfoTemplate<Object>("Object")
{
  DEF_OPENFDM_PROPERTY(String, Name, Serialized)
}

Object::ObjectTypeInfo::~ObjectTypeInfo(void)
{
}

Object::ObjectTypeInfo Object::sTypeInfo;

Object::Object(const std::string& name) :
  mName(name)
{
}

const TypeInfo&
Object::getTypeInfo(void) const
{
  return sTypeInfo;
}

bool
Object::getPropertyValue(const std::string& name, Variant& value) const
{
  if (!sTypeInfo.getPropertyValue(this, name, value))
    return false;
  return true;
}

bool
Object::setPropertyValue(const std::string& name, const Variant& value)
{
  if (!sTypeInfo.setPropertyValue(this, name, value))
    return false;
  return true;
}

void
Object::getPropertyInfoList(std::vector<PropertyInfo>& props) const
{
  sTypeInfo.getPropertyInfoList(props);
}

const std::string&
Object::getName(void) const
{
  return mName;
}

void
Object::setName(const std::string& name)
{
  mName = name;
}

const char* const
Object::getTypeName(void) const
{
  return getTypeInfo().getName();
}

Object*
Object::getUserData(void)
{
  return mUserData;
}

const Object*
Object::getUserData(void) const
{
  return mUserData;
}

void
Object::setUserData(Object* userData)
{
  mUserData = userData;
}

void
Object::destroy(const Object* object)
{
  delete object;
}

Object::~Object(void)
{
}

} // namespace OpenFDM
