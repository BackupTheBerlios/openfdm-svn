/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Object.h"
#include "Property.h"

namespace OpenFDM {

Object::Object(const std::string& name) :
  mName(name)
{
  addProperty("name", Property(this, &Object::getName, &Object::setName));
}

Object::~Object(void)
{
  std::list<Object**>::iterator it = _ptrList.begin();
  while (it != _ptrList.end()) {
    *(*it) = 0;
    ++it;
  }
}

const TypeInfo* const
Object::getTypeInfo(void) const
{
  return 0;
}

Property
Object::getProperty(const std::string& name)
{
  // Check if this one exists and return its value.
  if (0 < mProperties.count(name))
    return mProperties[name];
  else
    return Property();
}

std::list<std::string>
Object::listProperties(void) const
{
  std::list<std::string> nameList;
  PropertyMap::const_iterator it = mProperties.begin();
  while (it != mProperties.end()) {
    nameList.push_back((*it).first);
    ++it;
  }
  return nameList;
}

Variant
Object::getPropertyValue(const std::string& name) const
{
  // Just use the current property system for now

  // Return an empty variant if this property does not exist.
  if (mProperties.count(name) <= 0)
    return Variant();

  // Return the value of the property
  // FIXME: properties, like they are now, do not preserve constness
  return ((Property&)(mProperties.find(name)->second)).getValue();
}

void
Object::setPropertyValue(const std::string& name, const Variant& value)
{
  // Just use the current property system for now
  getProperty(name).setValue(value);
}

void
Object::addProperty(const std::string& name, const Property& property)
{
  mProperties[name] = property;
}

void
Object::removeProperty(const std::string& name)
{
  mProperties.erase(name);
}

void
Object::reg(Object** mp)
{
  _ptrList.push_back(mp);
}

void
Object::dereg(Object** mp)
{
  std::list<Object**>::iterator it = _ptrList.begin();
  while (it != _ptrList.end()) {
    if ((*it) == mp)
      it = _ptrList.erase(it);
    else
      ++it;
  }
}

} // namespace OpenFDM
