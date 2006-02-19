/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Object.h"

namespace OpenFDM {

Object::ObjectTypeInfo::ObjectTypeInfo(void) :
  TypeInfoTemplate<Object>("Object")
{
  DEF_OPENFDM_PROPERTY(String, Name, Serialized)
}

Object::ObjectTypeInfo::~ObjectTypeInfo(void)
{}

Object::ObjectTypeInfo Object::sTypeInfo;

Object::Object(const std::string& name) :
  mName(name)
{
}

Object::~Object(void)
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
  //// FIXME
  if (name != "Name")
    return false;
  value = getName();
  return true;
}

bool
Object::setPropertyValue(const std::string& name, const Variant& value)
{
  //// FIXME
  if (name != "Name")
    return false;
  setName(value.toString());
  return true;
}

void
Object::getPropertyInfoList(std::vector<PropertyInfo>& props) const
{
  sTypeInfo.getPropertyInfoList(props);
}

} // namespace OpenFDM
