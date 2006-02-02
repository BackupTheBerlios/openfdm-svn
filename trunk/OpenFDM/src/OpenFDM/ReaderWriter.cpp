/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Vehicle.h"
#include "Force.h"
#include "ReaderWriter.h"

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
