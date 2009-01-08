/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "EasyXMLReader.h"
#include <simgear/xml/easyxml.hxx>

namespace OpenFDM {
namespace XML {

class EasyXMLAttributes : public Attributes {
public:
  EasyXMLAttributes(const XMLAttributes* atts);
  virtual int getIndex(const char* qName) const;
  virtual int getLength(void) const;
  virtual const char* getLocalName(int index) const;
  virtual Type getType(int) const;
  virtual Type getType(const char*) const;
  virtual const char* getValue(int index) const;
  virtual const char* getValue(const char* qName) const;
private:
  const XMLAttributes* mAtts;
};

EasyXMLAttributes::EasyXMLAttributes(const XMLAttributes* atts) : mAtts(atts)
{
}

int
EasyXMLAttributes::getIndex(const char* qName) const
{
  return mAtts->findAttribute(qName);
}

int
EasyXMLAttributes::getLength(void) const
{
  return mAtts->size();
}

const char*
EasyXMLAttributes::getLocalName(int index) const
{
  return mAtts->getName(index);
}

Type
EasyXMLAttributes::getType(int) const
{
  return CDATA;
}

Type
EasyXMLAttributes::getType(const char*) const
{
  return CDATA;
}

const char*
EasyXMLAttributes::getValue(int index) const
{
  return mAtts->getValue(index);
}

const char*
EasyXMLAttributes::getValue(const char* qName) const
{
  return mAtts->getValue(qName);
}

class EasyXMLVisitor : public XMLVisitor {
public:
  EasyXMLVisitor(SharedPtr<ContentHandler> contentHandler,
                 SharedPtr<ErrorHandler> errorHandler) :
    mContentHandler(contentHandler),
    mErrorHandler(errorHandler)
  {}
  virtual ~EasyXMLVisitor(void) {}

  virtual void startXML()
  {
    if (!mContentHandler)
      return;
    mContentHandler->startDocument();
  }
  virtual void endXML()
  {
    if (!mContentHandler)
      return;
    mContentHandler->endDocument();
  }

  virtual void startElement(const char *name, const XMLAttributes& atts)
  {
    if (!mContentHandler)
      return;
    EasyXMLAttributes eAtts(&atts);
    mContentHandler->startElement("", name, name, &eAtts);
  }
  virtual void endElement(const char *name)
  {
    if (!mContentHandler)
      return;
    mContentHandler->endElement("", name, name);
  }
  virtual void data(const char *s, int length)
  {
    if (!mContentHandler)
      return;
    mContentHandler->characters(s, length);
  }
  virtual void pi(const char * target, const char * data)
  {
    if (!mContentHandler)
      return;
    mContentHandler->processingInstruction(target, data);
  }
  virtual void warning (const char * message, int line, int column)
  {
    if (!mErrorHandler)
      return;
    mErrorHandler->warning(message, line, column);
  }

private:
  SharedPtr<ContentHandler> mContentHandler;
  SharedPtr<ErrorHandler> mErrorHandler;
};


EasyXMLReader::EasyXMLReader(void)
{
}

EasyXMLReader::~EasyXMLReader(void)
{
}

void
EasyXMLReader::parse(std::istream& stream)
{
  try {
    const ErrorHandler* eh = mErrorHandler;
    const ContentHandler* ch = mContentHandler;
    EasyXMLVisitor easyXMLVisitor(const_cast<ContentHandler*>(ch),
                                  const_cast<ErrorHandler*>(eh));
    readXML(stream, easyXMLVisitor);
  } catch (const sg_exception& e) {
    if (mErrorHandler) {
      std::string error = e.getOrigin() + ": " + e.getFormattedMessage();
      const ErrorHandler* eh = mErrorHandler;
      const_cast<ErrorHandler*>(eh)->error(error.c_str(), 0, 0);
    }
  }
}

} // namespace XML
} // namespace OpenFDM
