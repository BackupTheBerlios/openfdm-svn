/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_XMLReader_H
#define OpenFDM_XMLReader_H

#include <string>
#include <iostream>
#include <list>
#include <stack>
#include <map>

#include <OpenFDM/Referenced.h>
#include <OpenFDM/SharedPtr.h>

#include <OpenFDM/XML/XMLReader.h>
#include <OpenFDM/XML/ContentHandler.h>
#include <OpenFDM/XML/ErrorHandler.h>
#include <OpenFDM/XML/Attributes.h>

#include <OpenFDM/XML/EasyXMLReader.h> // FIXME


namespace OpenFDM {

class XMLElement : public Referenced {
public:
  XMLElement(const char* n, const XML::Attributes* a);
  ~XMLElement(void);

  const std::string& getName(void) const;

  std::string getAttribute(const std::string& name) const;
  const std::map<std::string,std::string>& getAttributes(void) const;

  const std::string& getData(void) const;

  const XMLElement* getElement(const std::string& tagName) const;


  /// Returns the number if child elements with the given tagName.
  unsigned getNumElements(const std::string& tagName) const;

  std::list<const XMLElement*> getElements(const std::string& tagName) const;
  std::list<const XMLElement*> getElements(void) const;

  void appendData(const char* data, unsigned len);
  void appendChild(XMLElement* child);
private:
  std::string mName;
  std::string mData;
  std::map<std::string,std::string> mAttributes;
  std::list<SharedPtr<XMLElement> > mChildEntities;
};

class SimpleContentHandler : public XML::ContentHandler {
public:
  virtual ~SimpleContentHandler(void)
  {}
  virtual void characters(const char* data, unsigned length)
  { mElementStack.top()->appendData(data, length); }
  virtual void startDocument(void)
  { while (!mElementStack.empty()) mElementStack.pop(); mTopElement = 0; }
  virtual void endDocument(void)
  { while (!mElementStack.empty()) mElementStack.pop(); }
  virtual void startElement(const char* uri, const char* localName,
                            const char* qName, const XML::Attributes* atts)
  { mElementStack.push(new XMLElement(qName, atts)); }
  virtual void endElement(const char* uri, const char* localName,
                          const char* qName)
  {
    SharedPtr<XMLElement> current = mElementStack.top();
    mElementStack.pop();
    if (mElementStack.empty())
      mTopElement = current;
    else
      mElementStack.top()->appendChild(current);
  }

  SharedPtr<XMLElement> getTopElement(void) const
  { return mTopElement; }
private:
  SharedPtr<XMLElement> mTopElement;
  std::stack<SharedPtr<XMLElement> > mElementStack;
};

class SimpleErrorHandler : public XML::ErrorHandler {
public:
  virtual ~SimpleErrorHandler(void)
  {}
  virtual void error(const char* msg, unsigned line, unsigned col)
  { mMessage = msg; mLine = line; mColumn = col; }
  virtual void fatalError(const char* msg, unsigned line, unsigned col)
  { mMessage = msg; mLine = line; mColumn = col; }
  virtual void warning(const char* msg, unsigned line, unsigned col)
  { mMessage = msg; mLine = line; mColumn = col; }

  std::string mMessage;
  unsigned mLine;
  unsigned mColumn;
};

} // namespace OpenFDM

#endif
