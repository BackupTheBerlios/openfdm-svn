/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "ExpatXMLReader.h"
#include <iostream>
#include <expat.h>
#include "config.h"

namespace OpenFDM {
namespace XML {

class ExpatXMLAttributes : public Attributes {
public:
  ExpatXMLAttributes(const char**atts);
  virtual int getIndex(const char* qName) const;
  virtual int getLength(void) const;
  virtual const char* getLocalName(int index) const;
  virtual Type getType(int) const;
  virtual Type getType(const char*) const;
  virtual const char* getValue(int index) const;
  virtual const char* getValue(const char* qName) const;
private:
  unsigned mLength;
  const char** mAtts;
};

ExpatXMLAttributes::ExpatXMLAttributes(const char ** atts) :
  mLength(0), mAtts(atts)
{
  for (unsigned i = 0; mAtts[i] != 0; i += 2)
    ++mLength;
}

int
ExpatXMLAttributes::getIndex(const char* qName) const
{
  for (unsigned i = 0; mAtts[i] != 0; i += 2) {
    if (strcmp(mAtts[i*2], qName) == 0)
      return i;
  }
  return -1;
}

int
ExpatXMLAttributes::getLength(void) const
{
  return mLength;
}

const char*
ExpatXMLAttributes::getLocalName(int index) const
{
  return (index < mLength) ? mAtts[index*2] : 0;
}

Type
ExpatXMLAttributes::getType(int) const
{
  return CDATA; /*FIXME*/
}

Type
ExpatXMLAttributes::getType(const char*) const
{
  return CDATA; /*FIXME*/
}

const char*
ExpatXMLAttributes::getValue(int index) const
{
  return (index < mLength) ? mAtts[index*2+1] : 0;
}

const char*
ExpatXMLAttributes::getValue(const char* qName) const
{
  for (unsigned i = 0; mAtts[i] != 0; i += 2) {
    if (strcmp(mAtts[i*2], qName) == 0)
      return mAtts[i*2+1];
  }
  return 0;
}

static inline ContentHandler*
userDataToContentHandler(void* userData)
{
  ExpatXMLReader* reader = static_cast<ExpatXMLReader*>(userData);
  if (!reader)
    return 0;
  ContentHandler* contentHandler = reader->getContentHandler();
  return contentHandler;
}

static void
ExpatStartElement(void* userData, const char* name, const char** atts)
{
  ContentHandler* contentHandler = userDataToContentHandler(userData);
  ExpatXMLAttributes eAtts(atts);
  if (contentHandler)
    contentHandler->startElement("", name, name, &eAtts);
}

static void
ExpatEndElement(void* userData, const char* name)
{
  ContentHandler* contentHandler = userDataToContentHandler(userData);
  if (contentHandler)
    contentHandler->endElement("", name, name);
}

static void
ExpatCharacterData(void* userData, const char* data, int length)
{
  ContentHandler* contentHandler = userDataToContentHandler(userData);
  if (contentHandler)
    contentHandler->characters(data, length);
}

static void
ExpatComment(void* userData, const char* data)
{
  ContentHandler* contentHandler = userDataToContentHandler(userData);
  if (contentHandler)
    contentHandler->comment(data, strlen(data));
}

#ifdef HAVE_XML_SETSKIPPEDENTITYHANDLER
static void
ExpatSkippedEntity(void *userData, const char *entityName, int)
{
  ContentHandler* contentHandler = userDataToContentHandler(userData);
  if (contentHandler)
    contentHandler->skippedEntity(entityName);
}
#endif

static void
ExpatProcessingInstructions(void* userData, const char* target, const char* data)
{
  ContentHandler* contentHandler = userDataToContentHandler(userData);
  if (contentHandler)
    contentHandler->processingInstruction(target, data);
}

ExpatXMLReader::ExpatXMLReader(void)
{
}

ExpatXMLReader::~ExpatXMLReader(void)
{
}

void
ExpatXMLReader::parse(std::istream& stream)
{
  XML_Parser parser = XML_ParserCreate(0);
//   XML_Parser parser = XML_ParserCreateNS(0, ':');
  XML_SetUserData(parser, this);
  XML_SetElementHandler(parser, ExpatStartElement, ExpatEndElement);
  XML_SetCharacterDataHandler(parser, ExpatCharacterData);
  XML_SetProcessingInstructionHandler(parser, ExpatProcessingInstructions);
  XML_SetCommentHandler(parser, ExpatComment);
#ifdef HAVE_XML_SETSKIPPEDENTITYHANDLER
  XML_SetSkippedEntityHandler(parser, ExpatSkippedEntity);
#endif

  if (mContentHandler)
    mContentHandler->startDocument();

  unsigned bufSize = 32*1024;
  char* buf = new char[bufSize];
  while (!stream.eof()) {
    if (!stream.good()) {
      if (mErrorHandler)
        mErrorHandler->fatalError("ExpatXMLReader: "
                                  "Can not read from input stream",
                                  XML_GetCurrentLineNumber(parser),
                                  XML_GetCurrentColumnNumber(parser));
      XML_ParserFree(parser);
      delete [] buf;
      return;
    }

    stream.read(buf, bufSize);
    if (!XML_Parse(parser, buf, stream.gcount(), false)) {
      if (mErrorHandler)
        mErrorHandler->fatalError("ExpatXMLReader: Error from Parser",
                                  XML_GetCurrentLineNumber(parser),
                                  XML_GetCurrentColumnNumber(parser));
      XML_ParserFree(parser);
      delete [] buf;
      return;
    }
  }

  if (!XML_Parse(parser, buf, 0, true)) {
    if (mErrorHandler)
      mErrorHandler->fatalError("ExpatXMLReader: Error from Parser",
                                XML_GetCurrentLineNumber(parser),
                                XML_GetCurrentColumnNumber(parser));
    XML_ParserFree(parser);
    delete [] buf;
    return;
  }

  XML_ParserFree(parser);
  delete [] buf;

  if (mContentHandler)
    mContentHandler->endDocument();
}

} // namespace XML
} // namespace OpenFDM
