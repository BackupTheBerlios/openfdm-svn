/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_XML_XMLReader_H
#define OpenFDM_XML_XMLReader_H

#include <iosfwd>

#include <OpenFDM/Referenced.h>
#include <OpenFDM/SharedPtr.h>

#include "Attributes.h"
#include "ContentHandler.h"
#include "ErrorHandler.h"

namespace OpenFDM {
namespace XML {

class XMLReader : public Referenced {
public:
  virtual ~XMLReader(void);
  virtual void parse(std::istream& stream) = 0;

  ContentHandler* getContentHandler(void) const;
  void setContentHandler(ContentHandler* contentHandler);

  ErrorHandler* getErrorHandler(void) const;
  void setErrorHandler(ErrorHandler* errorHandler);

protected:
  SharedPtr<ContentHandler> mContentHandler;
  SharedPtr<ErrorHandler> mErrorHandler;
};

} // namespace XML
} // namespace OpenFDM

#endif
