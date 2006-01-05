/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_XMLReader_H
#define OpenFDM_XMLReader_H

#include <string>
#include <iostream>
#include <list>
#include <stack>
#include <map>

#include <simgear/xml/easyxml.hxx>

#include <OpenFDM/Object.h>

namespace OpenFDM {

class XMLDomParser;

class XMLElement
  : public Object {
public:
  typedef SharedPtr<XMLElement> pointer;
  typedef SharedPtr<const XMLElement> const_pointer;

  XMLElement(const std::string& name);
  virtual ~XMLElement(void);

  void setName(const std::string& name);
  const std::string& getName(void) const;

  void setAttribute(const std::string& name, const std::string& val);
  std::string getAttribute(const std::string& name) const;
  void removeAttribute(const std::string& name);
  const std::map<std::string,std::string>& getAttributes(void) const;

  const std::string& getData(void) const;
  void setData(const std::string& data);

  XMLElement* getElement(const std::string& tagName);
  const XMLElement* getElement(const std::string& tagName) const;


  /// Returns the number if child elements with the given tagName.
  unsigned getNumElements(const std::string& tagName) const;

  std::list<const_pointer> getElements(const std::string& tagName) const;

  std::list<pointer> getElements(const std::string& tagName);

  const std::list<pointer>& getElements(void) const;

  void appendChild(XMLElement* child);

private:
  std::string mName;
  std::string mData;
  std::map<std::string,std::string> mAttributes;
  std::list<pointer> mChildEntities;

  friend class XMLDomParser;
};

class XMLDocument
  : public Object {
public:
  typedef SharedPtr<XMLDocument> pointer;
  typedef SharedPtr<const XMLDocument> const_pointer;

  XMLElement* getElement(void);
  const XMLElement* getElement(void) const;
  void setElement(XMLElement* top);

private:
  SharedPtr<XMLElement> mTop;

  friend class XMLDomParser;
};

class XMLDomParser
  : public XMLVisitor {
public:
  XMLDomParser(void);
  virtual ~XMLDomParser(void);

  virtual void startElement(const char *name, const XMLAttributes& atts);
  virtual void endElement(const char *name);
  virtual void data(const char *s, int length);

  XMLDocument* getDocument(void)
  { return mDoc; }

  bool parseXML(std::istream& is);

  const std::string& getErrorMessage(void) const;

private:
  SharedPtr<XMLDocument> mDoc;
  std::stack<XMLElement::pointer> mElementStack;

  std::string mErrorMessage;
};

std::ostream&
operator<<(std::ostream& os, const XMLDocument& doc);

} // namespace OpenFDM

#endif
