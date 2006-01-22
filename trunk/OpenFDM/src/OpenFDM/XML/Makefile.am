INCLUDES = -I$(srcdir)/../..

OpenFDMXMLincludedir = $(includedir)/OpenFDM/XML

lib_LTLIBRARIES = libOpenFDMXML.la

nobase_OpenFDMXMLinclude_HEADERS = \
  Attributes.h \
  ContentHandler.h \
  ErrorHandler.h \
  XMLReader.h

libOpenFDMXML_la_SOURCES = \
  Attributes.cpp \
  ContentHandler.cpp \
  ErrorHandler.cpp \
  XMLReader.cpp

if ENABLE_EXPAT

lib_LTLIBRARIES += libOpenFDMexpatXML.la

nobase_OpenFDMXMLinclude_HEADERS += ExpatXMLReader.h
libOpenFDMexpatXML_la_SOURCES = ExpatXMLReader.cpp
libOpenFDMexpatXML_la_LIBADD = -lexpat libOpenFDMXML.la

endif

if ENABLE_EASYXML

lib_LTLIBRARIES += libOpenFDMeasyxmlXML.la

nobase_OpenFDMXMLinclude_HEADERS += EasyXMLReader.h
libOpenFDMeasyxmlXML_la_SOURCES = EasyXMLReader.cpp
libOpenFDMeasyxmlXML_la_LIBADD = -lsgxml -lsgstructure libOpenFDMXML.la

endif

if ENABLE_LIBXML2

lib_LTLIBRARIES += libOpenFDMlibxml2XML.la

nobase_OpenFDMXMLinclude_HEADERS += Libxml2XMLReader.h
libOpenFDMlibxml2XML_la_SOURCES = Libxml2XMLReader.cpp
libOpenFDMlibxml2XML_la_LIBADD = -lxml2 libOpenFDMXML.la

endif
