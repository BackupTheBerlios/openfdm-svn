/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_XML_ErrorHandler_H
#define OpenFDM_XML_ErrorHandler_H

#include <OpenFDM/Referenced.h>

namespace OpenFDM {
namespace XML {

class ErrorHandler : public Referenced {
public:
  virtual ~ErrorHandler(void);
  virtual void error(const char* msg, unsigned line, unsigned col);
  virtual void fatalError(const char* msg, unsigned line, unsigned col);
  virtual void warning(const char* msg, unsigned line, unsigned col);
};

} // namespace XML
} // namespace OpenFDM

#endif
