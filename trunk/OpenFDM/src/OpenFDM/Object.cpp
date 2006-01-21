/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Object.h"
#include "Property.h"

namespace OpenFDM {

Object::Object(const std::string& name) :
  mName(name)
{
  addStoredProperty("name", Property(this, &Object::getName, &Object::setName));
}

Object::~Object(void)
{
  std::list<Object**>::iterator it = _ptrList.begin();
  while (it != _ptrList.end()) {
    *(*it) = 0;
    ++it;
  }
}

const char*
Object::getTypeName(void) const
{
  return "Object";
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
  // Return an empty variant if this property does not exist.
  if (mProperties.count(name) <= 0)
    return Variant();

  // safety check
  Property property = mProperties.find(name)->second;
  if (!property.isValid())
    return Variant();

  // Return the value of the property
  return property.getValue();
}

void
Object::setPropertyValue(const std::string& name, const Variant& value)
{
  // Just use the current property system for now
  if (mProperties.count(name) <= 0)
    return;

  // safety check
  Property property = mProperties[name];
  if (!property.isValid())
    return;

  // set the property by the setter
  property.setValue(value);
}

bool
Object::isStoredProperty(const std::string& name) const
{
  // Return an empty variant if this property does not exist.
  if (mProperties.count(name) <= 0)
    return false;

  // safety check
  Property property = mProperties.find(name)->second;
  if (!property.isValid())
    return false;

  // Return the value of the property
  return property.isStoredProperty();
}

void
Object::addProperty(const std::string& name, const Property& property)
{
  mProperties[name] = property;
}

void
Object::addStoredProperty(const std::string& name, const Property& property)
{
  mProperties[name] = property;
  // FIXME, weak implementation
  mProperties[name].setStoredProperty(true);
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
