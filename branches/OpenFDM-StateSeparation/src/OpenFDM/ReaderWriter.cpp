/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "ReaderWriter.h"

#include "System.h"

namespace OpenFDM {

ReaderWriter::ReaderWriter(void)
{
  reset();
}

ReaderWriter::~ReaderWriter(void)
{
}

void
ReaderWriter::resetErrorState(void)
{
  mErrors.clear(); reset();
}

void
ReaderWriter::reset(void)
{
}

bool
ReaderWriter::error(const std::string& message)
{
  mErrors.push_back(message);
  return false;
}

} // namespace OpenFDM
