/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Object.h"
#include "Property.h"

namespace OpenFDM {

Object::Object(void)
{
}

Object::~Object(void)
{
  std::list<Object**>::iterator it = _ptrList.begin();
  while (it != _ptrList.end()) {
    *(*it) = 0;
    ++it;
  }
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
