INCLUDES = -I$(srcdir)/..

bin_PROGRAMS = jsb2openfdm
jsb2openfdm_SOURCES = jsb2openfdm.cpp
jsb2openfdm_LDFLAGS = 
jsb2openfdm_LDADD = libJSBReader.la ../OpenFDM/libOpenFDM.la

### FIXME!!!
OpenFDMincludedir = $(includedir)/JSBSim

OpenFDMinclude_HEADERS = \
  JSBSimAerosurfaceScale.h \
  JSBSimFCSComponent.h \
  JSBSimKinemat.h \
  JSBSimReader.h \
  JSBSimReaderBase.h \
  JSBSimScheduledGain.h \
  LegacyJSBSimReader.h \
  Tablereader.h \
  XMLReader.h

lib_LTLIBRARIES = libJSBReader.la
libJSBReader_la_SOURCES = \
  JSBSimAerosurfaceScale.cpp \
  JSBSimFCSComponent.cpp \
  JSBSimKinemat.cpp \
  JSBSimReader.cpp \
  JSBSimReaderBase.cpp \
  JSBSimScheduledGain.cpp \
  LegacyJSBSimReader.cpp \
  Tablereader.cpp \
  XMLReader.cpp
libJSBReader_la_LIBADD = ../OpenFDM/XML/libOpenFDMeasyxmlXML.la \
                         ../OpenFDM/XML/libOpenFDMXML.la \
                         ../OpenFDM/libOpenFDM.la
