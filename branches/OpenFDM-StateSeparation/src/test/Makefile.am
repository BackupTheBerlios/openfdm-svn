INCLUDES = -I$(srcdir)/..

# define some util programs
# bin_PROGRAMS = tiretestrig
bin_PROGRAMS = structure mechanic

# define some unit tests
#check_PROGRAMS = quattest unitstest structure simulationtime constintegral harmonic limit
check_PROGRAMS = quattest unitstest simulationtime limit
TESTS          = $(check_PROGRAMS)

# tiretestrig_SOURCES = tiretestrig.cpp
# tiretestrig_LDADD = ../OpenFDM/libOpenFDM.la

unitstest_SOURCES = unitstest.cpp
unitstest_LDADD = ../OpenFDM/libOpenFDM.la

quattest_SOURCES = quattest.cpp
quattest_LDADD = ../OpenFDM/libOpenFDM.la

mechanic_SOURCES = mechanic.cpp
mechanic_LDADD = ../OpenFDM/libOpenFDM.la

# foucault_SOURCES = foucault.cpp
# foucault_LDADD = ../OpenFDM/libOpenFDM.la

structure_SOURCES = structure.cpp
structure_LDADD = ../OpenFDM/libOpenFDM.la

simulationtime_SOURCES = simulationtime.cpp
simulationtime_LDADD = ../OpenFDM/libOpenFDM.la

# constintegral_SOURCES = constintegral.cpp
# constintegral_LDADD = ../OpenFDM/libOpenFDM.la

# harmonic_SOURCES = harmonic.cpp
# harmonic_LDADD = ../OpenFDM/libOpenFDM.la

limit_SOURCES = limit.cpp
limit_LDADD = ../OpenFDM/libOpenFDM.la
