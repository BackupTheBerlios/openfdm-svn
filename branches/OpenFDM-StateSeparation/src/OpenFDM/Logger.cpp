/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich
 *
 */

#include "LogStream.h"

#include <cstdlib>
#include <iostream>
#include <sstream>
#include "Mutex.h"
#include "ScopeLock.h"
#include "SharedPtr.h"

namespace OpenFDM {

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

void
Logger::setCategoryEnable(Logger::Category category, bool enable)
{
  Logger* logger = Instance();
  if (enable) {
    logger->mCategory |= category;
  } else {
    logger->mCategory &= ~category;
  }
}

void
Logger::setCategoryDisable(Logger::Category category)
{
  setCategoryEnable(category, false);
}

void
Logger::setPriority(Logger::Priority priority)
{
  Logger* logger = Instance();
  logger->mPriority = priority;
}

Logger*
Logger::Instance(void)
{
  static SharedPtr<Logger> ptr;
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

std::ostream*
Logger::getStream(Category category, Logger::Priority priority)
{
  if (!getEnabled(category, priority))
    return 0;

  if (mStream)
    return mStream;
  else if (Info <= priority)
    return &std::cout;
  else
    return &std::cerr;
}

Logger::Logger(std::ostream* stream) :
  mStream(stream),
  mCategory(~0u),
  mPriority(Error)
{
  // Set some defaults from the environment
  unsigned value = atou(std::getenv("OPENFDM_DEBUG_PRIORITY"));
  if (value)
    mPriority = value;

  value = atou(std::getenv("OPENFDM_DEBUG_CATEGORY"));
  if (value)
    mCategory = value;
}

} // namespace OpenFDM
