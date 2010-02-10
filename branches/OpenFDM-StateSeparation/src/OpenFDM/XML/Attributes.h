/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_XML_Attributes_H
#define OpenFDM_XML_Attributes_H

namespace OpenFDM {
namespace XML {

enum Type {
  CDATA,
  ID,
  IDREF,
  IDREFS,
  NMTOKEN,
  NMTOKENS,
  ENTITY,
  ENTITIES,
  NOTATION
};

class Attributes {
public:
  virtual ~Attributes(void);

  virtual int getIndex(const char* qName) const = 0;
//   virtual int getIndex(const char* uri, const char* localName) = 0;
  virtual int getLength(void) const = 0;
  virtual const char* getLocalName(int index) const = 0;
//   virtual const char* getQName(int index) const = 0;
  virtual Type getType(int index) const = 0;
  virtual Type getType(const char* qName) const = 0;
//   virtual Type getType(const char* uri, const char* localName) const = 0;
//   virtual const char* getURI(int index) const = 0;
  virtual const char* getValue(int index) const = 0;
  virtual const char* getValue(const char* qName) const = 0;
//   virtual const char* getValue(const char* uri, const char* localName) const = 0;
};

} // namespace XML
} // namespace OpenFDM

#endif
