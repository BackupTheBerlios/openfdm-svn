/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include <string>
#include <iostream>
#include <iomanip>
#include <list>
#include <map>
#include <stack>

#include "XMLReader.h"

namespace OpenFDM {

XMLElement::XMLElement(const std::string& name)
  : mName(name)
{
}

XMLElement::~XMLElement(void)
{
}

void
XMLElement::setName(const std::string& name)
{
  mName = name;
}

const std::string&
XMLElement::getName(void) const
{
  return mName;
}

void
XMLElement::setAttribute(const std::string& name, const std::string& val)
{
  mAttributes[name] = val;
}

std::string
XMLElement::getAttribute(const std::string& name) const
{
  if (0 < mAttributes.count(name))
    return mAttributes.find(name)->second;
  else
    return std::string();
}

void
XMLElement::removeAttribute(const std::string& name)
{
  mAttributes.erase(name);
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

void
XMLElement::setData(const std::string& data)
{
  mData = data;
}

XMLElement*
XMLElement::getElement(const std::string& tagName)
{
  std::list<pointer>::iterator it = mChildEntities.begin();
  while (it != mChildEntities.end()) {
    if ((*it)->mName == tagName) {
      return *it;
    }
    ++it;
  }
  return 0;
}

const XMLElement*
XMLElement::getElement(const std::string& tagName) const
{
  std::list<pointer>::const_iterator it = mChildEntities.begin();
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
  std::list<pointer>::const_iterator it = mChildEntities.begin();
  while (it != mChildEntities.end()) {
    if ((*it)->mName == tagName) {
      ++num;
    }
    ++it;
  }
  return num;
}

std::list<XMLElement::const_pointer>
XMLElement::getElements(const std::string& tagName) const
{
  std::list<const_pointer> ret;
  std::list<pointer>::const_iterator it = mChildEntities.begin();
  while (it != mChildEntities.end()) {
    if ((*it)->mName == tagName) {
      ret.push_back(*it);
    }
    ++it;
  }
  return ret;
}

std::list<XMLElement::pointer>
XMLElement::getElements(const std::string& tagName)
{
  std::list<pointer> ret;
  std::list<pointer>::iterator it = mChildEntities.begin();
  while (it != mChildEntities.end()) {
    if ((*it)->mName == tagName) {
      ret.push_back(*it);
    }
    ++it;
  }
  return ret;
}

const std::list<XMLElement::pointer>&
XMLElement::getElements(void) const
{
  return mChildEntities;
}

void
XMLElement::appendChild(XMLElement* child)
{
  mChildEntities.push_back(child);
}

XMLElement*
XMLDocument::getElement(void)
{
  return mTop;
}

const XMLElement*
XMLDocument::getElement(void) const
{
  return mTop;
}

void
XMLDocument::setElement(XMLElement* top)
{
  mTop = top;
}

XMLDomParser::XMLDomParser(void)
{
}

XMLDomParser::~XMLDomParser(void)
{
}

void
XMLDomParser::startElement(const char *name, const XMLAttributes& atts)
{
  XMLElement::pointer element(new XMLElement(name));

  // Set the attributes.
  int size = atts.size();
  for (int i = 0; i < size; ++i)
    element->setAttribute(atts.getName(i), atts.getValue(i));

  // Attach to this tag.
  if (mElementStack.empty()) {
    if (mDoc)
      std::cerr << "Warning more than one toplevel entity!!!" << std::endl;
    mDoc = XMLDocument::pointer(new XMLDocument);
    mDoc->setElement(element);
  } else
    mElementStack.top()->appendChild(element);
  
  // Push on top of the stack.
  mElementStack.push(element);
}

void
XMLDomParser::endElement(const char *name)
{
  mElementStack.pop();
}

void
XMLDomParser::data(const char *s, int length)
{
  mElementStack.top()->mData += std::string(s, length);
}

bool
XMLDomParser::parseXML(std::istream& is)
{
  bool success = false;
  try {
    readXML(is, *this);
    mErrorMessage = std::string();
    success = true;
  } catch (const sg_exception& e) {
    mErrorMessage = e.getOrigin() + ": " + e.getFormattedMessage();
  }
  return success;
}

const std::string&
XMLDomParser::getErrorMessage(void) const
{
  return mErrorMessage;
}

static std::ostream&
printElement(std::ostream& os, XMLElement::const_pointer element, int indent = 0)
{
  bool emptyData = element->getData() == "";

  os << '<' << element->getName();
  std::map<std::string,std::string>::const_iterator
    mit = element->getAttributes().begin();
  while (mit != element->getAttributes().end()) {
    os << ' ' << (*mit).first << "=\"" << (*mit).second << "\"";
    ++mit;
  }
  os << '>';

  if (emptyData)
    os << std::endl;

  std::list<XMLElement::pointer>::const_iterator
    it = element->getElements().begin();
  while (it != element->getElements().end()) {
    if (emptyData)
      os << std::setw(indent+2) << "";
    printElement(os, *it, indent + 2);
    if (emptyData)
      os << std::endl;
    ++it;
  }
  
  if (emptyData)
    os << std::setw(indent) << "";

  os << element->getData() << "</" << element->getName() << '>';

  return os;
}

std::ostream&
operator<<(std::ostream& os, const XMLDocument& doc)
{
  os << "<?xml version=\"1.0\"?>" << std::endl;
  return printElement(os, doc.getElement()) << std::endl;
}

} // namespace OpenFDM
