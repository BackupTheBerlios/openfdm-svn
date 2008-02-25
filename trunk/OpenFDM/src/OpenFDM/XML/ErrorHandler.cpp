/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "ErrorHandler.h"

namespace OpenFDM {
namespace XML {

ErrorHandler::~ErrorHandler(void)
{
}

void
ErrorHandler::error(const char* msg, unsigned line, unsigned col)
{
}

void
ErrorHandler::fatalError(const char* msg, unsigned line, unsigned col)
{
}

void
ErrorHandler::warning(const char* msg, unsigned line, unsigned col)
{
}

} // namespace XML
} // namespace OpenFDM
