/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Assert_H
#define OpenFDM_Assert_H

#include "OpenFDMConfig.h"

// FIXME, appears not to work currently ...
#if !defined(NDEBUG) //&& (OpenFDM_ENABLE_DEBUG == 1)
# define OpenFDMDebug
#endif

#if !defined(NDEBUG) && (OpenFDM_ENABLE_RANGE_CHECKING == 1)
# define OpenFDMLinAlgDebug
#endif

#ifdef OpenFDMDebug
# define OpenFDMAssert(a)                           \
do {                                                \
  if (!(a)) {                                       \
    ::OpenFDM::Assertion(__FILE__, __LINE__, #a );  \
  }                                                 \
} while (0)
#else
# define OpenFDMAssert(a) ((void)(0))
#endif

#ifdef OpenFDMLinAlgDebug
# define OpenFDMLinAlgAssert(a)                     \
do {                                                \
  if (!(a)) {                                       \
    ::OpenFDM::Assertion(__FILE__, __LINE__, #a );  \
  }                                                 \
} while (0)
#else
# define OpenFDMLinAlgAssert(a) ((void)(0))
#endif

#define OpenFDMError(a) ::OpenFDM::Assertion(__FILE__, __LINE__, a );

namespace OpenFDM {

void Assertion(const char* file, int line, const char* condition);

} // namespace OpenFDM

#endif
