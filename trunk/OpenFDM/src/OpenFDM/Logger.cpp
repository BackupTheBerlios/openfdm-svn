/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include <cstdlib>
#include <iostream>
#include <sstream>

#include "Mutex.h"
#include "ScopeLock.h"
#include "LogStream.h"

namespace OpenFDM {

namespace Log {

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

Logger::Logger(std::basic_ostream<char>* stream) :
  mStream(stream),
  mCategory(~0u),
  mPriority(Log::Error)
{
  // Set some defaults from the environment
  unsigned value = atou(std::getenv("OPENFDM_DEBUG_PRIORITY"));
  if (value)
    mPriority = value;
      
  value = atou(std::getenv("OPENFDM_DEBUG_CATEGORY"));
  if (value)
    mCategory = value;
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

Logger*
Logger::Instance(void)
{
  static Logger* ptr = 0;
  if (!ptr) {
    static Mutex mutex;
    ScopeLock scopeLock(mutex);
    if (!ptr) {
      // Create new instance ...
      ptr = new Logger(&std::cerr);
    }
  }
  return ptr;
}

} // namespace Log

} // namespace OpenFDM
