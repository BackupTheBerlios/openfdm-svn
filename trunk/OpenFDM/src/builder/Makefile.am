
qtinclude=$(QTDIR)/include
othrinclude=$(HOME)/sw/include
osginclude=$(HOME)/sw/include
x11lib=/usr/X11R6/lib

qtlib=$(QTDIR)/lib
othrlib=$(HOME)/sw/lib
osglib=$(HOME)/sw/lib

INCLUDES = -I$(srcdir)/.. -I$(qtinclude) -I$(othrinclude) -I$(osginclude)

bin_PROGRAMS = openfdm

openfdm_SOURCES = \
	main.cpp \
	Configuration.h \
	Configuration.cpp \
	FrameItem.h \
	FrameItem.cpp \
	Geometries.h \
	Geometries.cpp \
	MainWindow.h \
	MainWindow.cpp \
	ModelItem.h \
	ModelItem.cpp \
	QOsgWidget.h \
	QOsgWidget.cpp

openfdm_LDADD = ../JSBSim/libOpenFDMJSBReader.la ../OpenFDM/libOpenFDM.la \
                -losgGA -losgUtil -losgDB -losg -lOpenThreads \
                -lQtGui -lQtOpenGL -lQtCore \
                -lGL -lGLU

openfdm_LDFLAGS = -L$(qtlib) \
                -L$(osglib) -L$(othrlib) -Wl,-rpath,$(osglib) \
                -L$(qtlib)
