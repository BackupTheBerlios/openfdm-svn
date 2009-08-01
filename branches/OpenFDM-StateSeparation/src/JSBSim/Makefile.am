INCLUDES = -I$(srcdir)/..

bin_PROGRAMS = jsb2openfdm
jsb2openfdm_SOURCES = jsb2openfdm.cpp
jsb2openfdm_LDFLAGS = 
jsb2openfdm_LDADD = libOpenFDMJSBReader.la ../OpenFDM/libOpenFDM.la

### FIXME!!!
OpenFDMincludedir = $(includedir)/JSBSim

OpenFDMinclude_HEADERS = \
  JSBSimAerodynamic.h \
  JSBSimAerosurfaceScale.h \
  JSBSimFCSComponent.h \
  JSBSimKinemat.h \
  JSBSimReader.h \
  JSBSimReaderBase.h \
  JSBSimScheduledGain.h \
  Tablereader.h \
  XMLReader.h

lib_LTLIBRARIES = libOpenFDMJSBReader.la
libOpenFDMJSBReader_la_SOURCES = \
  JSBSimAerodynamic.cpp \
  JSBSimAerosurfaceScale.cpp \
  JSBSimFCSComponent.cpp \
  JSBSimKinemat.cpp \
  JSBSimReader.cpp \
  JSBSimReaderBase.cpp \
  JSBSimScheduledGain.cpp \
  Tablereader.cpp \
  XMLReader.cpp
libOpenFDMJSBReader_la_LIBADD = ../OpenFDM/XML/libOpenFDMeasyxmlXML.la \
                                ../OpenFDM/XML/libOpenFDMXML.la \
                                ../OpenFDM/libOpenFDM.la
