INCLUDES = -I$(srcdir)/..

noinst_PROGRAMS = main
main_SOURCES = main.cpp
main_LDFLAGS = 
main_LDADD = libJSBReader.la ../OpenFDM/libOpenFDM.la -L$(HOME)/sw/lib -lsgxml -lsgstructure

### FIXME!!!
OpenFDMincludedir = $(includedir)/JSBSim

OpenFDMinclude_HEADERS = \
  JSBReader.h

lib_LTLIBRARIES = libJSBReader.la
libJSBReader_la_SOURCES = \
  JSBReader.cpp
