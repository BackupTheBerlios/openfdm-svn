# -*- Autoconf -*-

AC_PREREQ(2.59)
AC_INIT([OpenFDM], [0.5])
dnl AC_CONFIG_AUX_DIR(config)
AC_CONFIG_SRCDIR([src/OpenFDM/Object.h])
dnl AC_CONFIG_HEADER([config.h])
AM_INIT_AUTOMAKE()

dnl AC_CANONICAL_HOST
AC_LANG(C++)
AC_PROG_CC
AC_PROG_CPP
AC_PROG_CXX
AC_PROG_INSTALL

AC_DISABLE_SHARED
AC_PROG_LIBTOOL
AC_SUBST(LIBTOOL_DEPS)

dnl try to append the prefix path to the include and library paths if present
if test "x$prefix" != "xNONE" ; then
  CPPFLAGS="$CPPFLAGS -I$prefix/include"
  LDFLAGS="$LDFLAGS -L$prefix/lib"
fi

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

dnl Choice of used xml parsers
dnl Check for expat
AC_CHECK_HEADERS([expat.h],
  [AC_CHECK_LIB([expat],[XML_ExpatVersion],
     [openfdm_expat_enabled="yes"])
  ])
AM_CONDITIONAL([ENABLE_EXPAT], [test "x$openfdm_expat_enabled" = "xyes"])
if test "x$openfdm_expat_enabled" = "xyes"
then
  save_LIBS=$LIBS
  LIBS=-lexpat
  AC_CHECK_FUNC([XML_SetSkippedEntityHandler],
    AC_DEFINE([HAVE_XML_SETSKIPPEDENTITYHANDLER], [1],
              [Define if expat provides XML_SetSkippedEntityHandler]))
  LIBS=$save_LIBS

  OpenFDM_HAVE_EXPAT_XMLREADER=1
else
  OpenFDM_HAVE_EXPAT_XMLREADER=0
fi
AC_SUBST(OpenFDM_HAVE_EXPAT_XMLREADER)

dnl Check for SimGear and easyxml
AC_CHECK_HEADERS([simgear/xml/easyxml.hxx],[openfdm_easyxml_enabled="yes"])
AM_CONDITIONAL([ENABLE_EASYXML], [test "x$openfdm_easyxml_enabled" = "xyes"])
if test "x$openfdm_easyxml_enabled" = "xyes"
then
  OpenFDM_HAVE_EASYXML_XMLREADER=1
else
  OpenFDM_HAVE_EASYXML_XMLREADER=0
fi
AC_SUBST(OpenFDM_HAVE_EASYXML_XMLREADER)

dnl Check for libxml2 - this is the prefered one, since it does validation
dnl not yet implemented
AM_CONDITIONAL([ENABLE_LIBXML2], [test "x$openfdm_libxml2_enabled" = "xyes"])
if test "x$openfdm_libxml2_enabled" = "xyes"
then
  OpenFDM_HAVE_LIBXML2_XMLREADER=1
else
  OpenFDM_HAVE_LIBXML2_XMLREADER=0
fi
AC_SUBST(OpenFDM_HAVE_LIBXML2_XMLREADER)

dnl simple check for qt4
if test "x$QTDIR" != "x" ; then
  CPPFLAGS="$CPPFLAGS -I$QTDIR/include"
  LDFLAGS="$LDFLAGS -L$QTDIR/lib"
  AC_CHECK_HEADERS([Qt/QtCore],[openfdm_have_qt4="yes"])
  QT_MOC=$QTDIR/bin/moc
  QT_UIC=$QTDIR/bin/uic
else
  AC_MSG_WARN([You must have QTDIR set to the qt-4 directory for the builder])
  QT_MOC=/bin/false
  QT_UIC=/bin/false
fi
AC_SUBST(QT_MOC)
AC_SUBST(QT_UIC)

dnl simple check for osg
dnl AC_CHECK_HEADERS([osg/Version],[openfdm_have_osg="yes"])
openfdm_have_osg="no"

dnl avoid trying to build the editor if qt4 or osg is missing
if test "x$openfdm_have_qt4" = "xyes" && test "x$openfdm_have_osg" = "xyes"
then
  openfdm_builder_enabled="yes"
else
  openfdm_builder_enabled="no"
fi
AM_CONDITIONAL([ENABLE_BUILDER], [test "x$openfdm_builder_enabled" = "xyes"])

dnl write makefiles and config.h
AM_CONFIG_HEADER([src/OpenFDM/config.h])
AC_CONFIG_FILES([src/OpenFDM/OpenFDMConfig.h])
AC_OUTPUT([
  Makefile
  src/Makefile
  src/OpenFDM/Makefile
  src/OpenFDM/XML/Makefile
  src/JSBSim/Makefile
  src/test/Makefile
  src/builder/Makefile
  share/Makefile
  share/objects/Makefile
])

