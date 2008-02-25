/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Assert.h"

#ifdef HAVE_CONFIG_H
# include "config.h"
#endif

#include "LogStream.h"

#ifdef HAVE_EXECINFO_H
#include <execinfo.h>
#endif
#ifdef HAVE_CXXABI_H
#include <cxxabi.h>
#include <string.h>
#endif

#include <iostream>
#include <cstdlib>

namespace OpenFDM {

void Assertion(const char* file, int line, const char* condition)
{
  Log(Assert, Error) << "Assertion OpenFDMAssert(" << condition
                     << ") failed at line "
                     << line << " in " << file << endl;

#if defined(HAVE_BACKTRACE_SYMBOLS) && defined(HAVE_BACKTRACE)

#define BACKTRACE_SIZE 10
  void *array[1 + BACKTRACE_SIZE];

  size_t size = backtrace(array, 1 + BACKTRACE_SIZE);
  char **strings = backtrace_symbols(array, size);

  Log(Assert, Error) << " Backtrace:" << endl;

  for (size_t i = 1; i < size; ++i) {
    Log(Assert, Error) << "  " << strings[i] << endl;
#ifdef HAVE_CXXABI_H
    char *mangled = ::strchr(strings[i], '(');
    char *rest = 0;
    if (mangled) {
      rest = ::strchr(mangled+1, '+');
      if (!rest)
        rest = ::strchr(mangled+1, ')');
    }        
     
    if (mangled && rest && rest-mangled < 1024) {
      char manglebuf[1024];
      ::memcpy(manglebuf, mangled+1, rest-mangled-1);
      manglebuf[rest-mangled-1] = '\0';
      size_t length = 1024;
      char demangled[1024];
      int status = 0;
      abi::__cxa_demangle(manglebuf, demangled, &length, &status);
      if (status == 0)
        Log(Assert, Error) << "    (" << demangled << ")" << endl;
    }
#endif
  }

  free (strings);
#endif
}

} // namespace OpenFDM

