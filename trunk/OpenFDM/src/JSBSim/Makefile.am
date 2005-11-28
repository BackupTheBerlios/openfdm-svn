INCLUDES = -I$(srcdir)/..

bin_PROGRAMS = jsb2openfdm
jsb2openfdm_SOURCES = jsb2openfdm.cpp
jsb2openfdm_LDFLAGS = 
jsb2openfdm_LDADD = libJSBReader.la ../OpenFDM/libOpenFDM.la

### FIXME!!!
OpenFDMincludedir = $(includedir)/JSBSim

OpenFDMinclude_HEADERS = \
  JSBSimFCSComponent.h \
  JSBSimAerosurfaceScale.h \
  JSBSimKinemat.h \
  JSBSimScheduledGain.h \
  LegacyJSBSimReader.h

lib_LTLIBRARIES = libJSBReader.la
libJSBReader_la_SOURCES = \
  JSBSimFCSComponent.cpp \
  JSBSimAerosurfaceScale.cpp \
  JSBSimKinemat.cpp \
  JSBSimScheduledGain.cpp \
  LegacyJSBSimReader.cpp
