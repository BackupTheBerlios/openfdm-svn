INCLUDES = -I$(srcdir)/..

bin_PROGRAMS = openfdm

DISTSOURCES = \
	main.cpp \
	Configuration.cpp \
	FrameItem.cpp \
	Geometries.cpp \
	MainWindow.cpp \
	ModelItem.cpp \
	QOsgWidget.cpp

# Headers with QObjects
DISTHEADERS_MOC = \
	QOsgWidget.h

# Headers without QObjects
DISTHEADERS_NO_MOC = \
	Configuration.h \
	FrameItem.h \
	Geometries.h \
	MainWindow.h \
	ModelItem.h

# ui files
FORMS = 

# The derived files
FORMHEADERS = $(FORMS:.ui=.h)
MOC_CC = $(FORMS:.ui=.moc.cpp) $(DISTHEADERS_MOC:.h=.moc.cpp)
UIC_CC = $(FORMS:.ui=.uic.cpp)

BUILT_SOURCES = $(FORMHEADERS) $(UIC_CC) $(MOC_CC)
CLEANFILES = $(BUILT_SOURCES)

openfdm_SOURCES = $(DISTSOURCES) $(DISTHEADERS_MOC) $(DISTHEADERS_NO_MOC)
nodist_openfdm_SOURCES = $(MOC_CC) $(UIC_CC)

openfdm_LDADD = ../JSBSim/libOpenFDMJSBReader.la ../OpenFDM/libOpenFDM.la \
                -losgGA -losgUtil -losgDB -losg -lOpenThreads \
                -lQtGui -lQtOpenGL -lQtCore \
                -lGL -lGLU

# Some suffix rules to make qt built sources
.ui.h:
	$(QT_UIC) -o $@ $<

.h.moc.cpp:
	$(QT_MOC) -o $@ $<

.ui.uic.cpp:
	$(QT_UIC) -o $@ -impl $*.h $<

SUFFIXES = .h .ui .moc.cpp .uic.cpp
