AC_INIT([OpenFDM], [0.5])
dnl AC_PREREQ(2.59)
dnl AC_COPYRIGHT(COPYRIGHT-NOTICE)
AM_INIT_AUTOMAKE

AC_CONFIG_SRCDIR([src/OpenFDM/Object.h])

CPPFLAGS="$CPPFLAGS -I$prefix/include"
LDFLAGS="$LDFLAGS -L$prefix/lib"

AC_CANONICAL_HOST

AC_PROG_CC
AC_PROG_CPP
AC_PROG_CXX

AC_DISABLE_SHARED
AC_PROG_LIBTOOL
AC_SUBST(LIBTOOL_DEPS)

dnl Configure debugging checks
AC_ARG_ENABLE(debug,
[  --enable-debug            Enable assertions [default=yes]],
[enable_debug="$enableval"],[enable_debug="yes"])
[
case $enable_debug in
  yes) enable_debug=1 ;;
  *) enable_debug=0 ;;
esac
]
AC_SUBST(enable_debug)

dnl Configure range checking
AC_ARG_ENABLE(range-checking,
[  --enable-range-checking   Enable matrix/vector range checking [default=no]],
[enable_range_checking="$enableval"],[enable_range_checking="no"])
[
case $enable_range_checking in
  yes) enable_range_checking=1 ;;
  *) enable_range_checking=0 ;;
esac
]
AC_SUBST(enable_range_checking)

dnl Check for headers and symbols required for better speaking backtraces ...
AC_CHECK_HEADERS([execinfo.h cxxabi.h])
AC_CHECK_FUNCS([backtrace backtrace_symbols])

AM_CONFIG_HEADER([src/OpenFDM/config.h])

dnl write makefiles and config.h
AC_OUTPUT([
  src/OpenFDM/OpenFDMConfig.h
  Makefile
  src/Makefile
  src/OpenFDM/Makefile
  src/JSBSim/Makefile
  src/builder/Makefile
  share/Makefile
  share/objects/Makefile
])

