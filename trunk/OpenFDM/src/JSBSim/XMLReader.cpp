/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "XMLReader.h"

#include <string>
#include <iostream>
#include <iomanip>
#include <list>
#include <map>
#include <stack>

#include <OpenFDM/XML/XMLReader.h>
#include <OpenFDM/XML/ContentHandler.h>
#include <OpenFDM/XML/ErrorHandler.h>
#include <OpenFDM/XML/Attributes.h>

#include <OpenFDM/XML/EasyXMLReader.h> // FIXME

namespace OpenFDM {

XMLElement::XMLElement(const char* name, const XML::Attributes* a)
  : mName(name)
{
  unsigned nAtts = a->getLength();
  for (unsigned i = 0; i < nAtts; ++i)
    mAttributes[a->getLocalName(i)] = a->getValue(i);
}

XMLElement::~XMLElement(void)
{
}

const std::string&
XMLElement::getName(void) const
{
  return mName;
}

std::string
XMLElement::getAttribute(const std::string& name) const
{
  if (0 < mAttributes.count(name))
    return mAttributes.find(name)->second;
  else
    return std::string();
}

const std::map<std::string,std::string>&
XMLElement::getAttributes(void) const
{
  return mAttributes;
}

const std::string&
XMLElement::getData(void) const
{
  return mData;
}

const XMLElement*
XMLElement::getElement(const std::string& tagName) const
{
  std::list<SharedPtr<XMLElement> >::const_iterator it = mChildEntities.begin();
  while (it != mChildEntities.end()) {
    if ((*it)->mName == tagName) {
      return *it;
    }
    ++it;
  }
  return 0;
}

unsigned
XMLElement::getNumElements(const std::string& tagName) const
{
  unsigned num = 0;
  std::list<SharedPtr<XMLElement> >::const_iterator it = mChildEntities.begin();
  while (it != mChildEntities.end()) {
    if ((*it)->mName == tagName) {
      ++num;
    }
    ++it;
  }
  return num;
}

std::list<const XMLElement*>
XMLElement::getElements(const std::string& tagName) const
{
  std::list<const XMLElement*> ret;
  std::list<SharedPtr<XMLElement> >::const_iterator it = mChildEntities.begin();
  while (it != mChildEntities.end()) {
    if ((*it)->mName == tagName) {
      ret.push_back(*it);
    }
    ++it;
  }
  return ret;
}

std::list<const XMLElement*>
XMLElement::getElements(void) const
{
  return std::list<const XMLElement*>(mChildEntities.begin(),
                                      mChildEntities.end());
}

void
XMLElement::appendData(const char* data, unsigned len)
{
  mData.append(data, len);
}

void
XMLElement::appendChild(XMLElement* child)
{
  mChildEntities.push_back(child);
}

} // namespace OpenFDM
