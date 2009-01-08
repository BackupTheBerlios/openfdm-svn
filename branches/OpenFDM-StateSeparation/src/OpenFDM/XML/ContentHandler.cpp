/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "ContentHandler.h"

namespace OpenFDM {
namespace XML {

ContentHandler::~ContentHandler(void)
{
}

void
ContentHandler::characters(const char* data, unsigned length)
{
}

void
ContentHandler::comment(const char* commentData, unsigned length)
{
}

void
ContentHandler::startDocument(void)
{
}

void
ContentHandler::endDocument(void)
{
}

void
ContentHandler::startElement(const char*, const char*, const char*,
                             const Attributes*)
{
}

void
ContentHandler::endElement(const char*, const char*, const char*)
{
}

void
ContentHandler::skippedEntity(const char* name)
{
}

void
ContentHandler::processingInstruction(const char*, const char*)
{
}

} // namespace XML
} // namespace OpenFDM
