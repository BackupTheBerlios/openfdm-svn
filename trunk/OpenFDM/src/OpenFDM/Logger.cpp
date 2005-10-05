/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include <cstdlib>
#include <iostream>
#include <sstream>
#include "LogStream.h"

namespace OpenFDM {

namespace Log {

Logger::Logger(std::basic_ostream<char>* stream) :
  mStream(stream),
  mCategory(~0u),
  mPriority(Log::Error)
{
}

void
Logger::setCategoryEnable(Log::Category category, bool enable)
{
  Logger* logger = Instance();
  if (enable) {
    logger->mCategory |= category;
  } else {
    logger->mCategory &= ~category;
  }
}

void
Logger::setCategoryDisable(Log::Category category)
{
  setCategoryEnable(category, false);
}

void
Logger::setPriority(Log::Priority priority)
{
  Logger* logger = Instance();
  logger->mPriority = priority;
}

static unsigned
atou(const char* s)
{
  if (!s)
    return 0u;

  std::stringstream strstream(s);
  unsigned value;
  strstream >> value;
  if (!strstream)
    return 0u;

  return value;
}

Logger*
Logger::Instance(void)
{
  static Logger* ptr = 0;
  if (!ptr) {
    // Create new instance ...
    ptr = new Logger(&std::cerr);
    
    // ... and set some defaults from the environment.
    unsigned value = atou(std::getenv("OPENFDM_DEBUG_PRIORITY"));
    if (value)
      ptr->mPriority = value;

    value = atou(std::getenv("OPENFDM_DEBUG_CATEGORY"));
    if (value)
      ptr->mCategory = value;
  }
  return ptr;
}

} // namespace Log

} // namespace OpenFDM
