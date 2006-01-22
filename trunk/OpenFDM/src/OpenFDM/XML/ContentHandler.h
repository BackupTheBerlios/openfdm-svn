/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_XML_ContentHandler_H
#define OpenFDM_XML_ContentHandler_H

#include <OpenFDM/Referenced.h>

namespace OpenFDM {
namespace XML {

class Attributes;

class ContentHandler : public Referenced {
public:
  virtual ~ContentHandler(void);

  virtual void characters(const char* data, unsigned length);
/*   virtual void ignorableWhitespace(const char* data, unsigned length); */

  virtual void comment(const char* commentData, unsigned length);

  virtual void startDocument(void);
  virtual void endDocument(void);

  virtual void startElement(const char* uri, const char* localName,
                            const char* qName, const Attributes* atts);
  virtual void endElement(const char* uri, const char* localName,
                          const char* qName);

  virtual void skippedEntity(const char* name);
  virtual void processingInstruction(const char* target, const char* data);

//   virtual void startPrefixMapping(const char* prefix, const char* uri);
//   virtual void endPrefixMapping(const char* prefix);

//   virtual void setDocumentLocator(Locator locator);
};

} // namespace XML
} // namespace OpenFDM

#endif
