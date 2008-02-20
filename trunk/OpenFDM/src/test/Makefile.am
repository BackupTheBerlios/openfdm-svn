INCLUDES = -I$(srcdir)/..

# define some unit tests
check_PROGRAMS = quattest unitstest simulationtime constintegral harmonic
TESTS          = $(check_PROGRAMS)

unitstest_SOURCES = unitstest.cpp
unitstest_LDADD = ../OpenFDM/libOpenFDM.la

quattest_SOURCES = quattest.cpp
quattest_LDADD = ../OpenFDM/libOpenFDM.la

simulationtime_SOURCES = simulationtime.cpp
simulationtime_LDADD = ../OpenFDM/libOpenFDM.la

constintegral_SOURCES = constintegral.cpp
constintegral_LDADD = ../OpenFDM/libOpenFDM.la

harmonic_SOURCES = harmonic.cpp
harmonic_LDADD = ../OpenFDM/libOpenFDM.la
