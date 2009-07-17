INCLUDES = -I$(srcdir)/..

# define some unit tests
TESTS = \
	constintegral \
	harmonic \
	limit \
	linalg \
	quattest \
	simulationtime \
	structure \
	unitstest

# define some util programs
noinst_PROGRAMS = \
	ballistic \
	drop \
	foucault \
	inertianav \
	mechanic \
	structure \
	tiretestrig \
	coordsys \
	$(TESTS)

ballistic_SOURCES = ballistic.cpp
ballistic_LDADD = ../OpenFDM/libOpenFDM.la

constintegral_SOURCES = constintegral.cpp
constintegral_LDADD = ../OpenFDM/libOpenFDM.la

coordsys_SOURCES = coordsys.cpp
coordsys_LDADD = ../OpenFDM/libOpenFDM.la

drop_SOURCES = drop.cpp
drop_LDADD = ../OpenFDM/libOpenFDM.la

foucault_SOURCES = foucault.cpp
foucault_LDADD = ../OpenFDM/libOpenFDM.la

harmonic_SOURCES = harmonic.cpp
harmonic_LDADD = ../OpenFDM/libOpenFDM.la

inertianav_SOURCES = inertianav.cpp
inertianav_LDADD = ../OpenFDM/libOpenFDM.la

limit_SOURCES = limit.cpp
limit_LDADD = ../OpenFDM/libOpenFDM.la

linalg_SOURCES = linalg.cpp
linalg_LDADD = ../OpenFDM/libOpenFDM.la

mechanic_SOURCES = mechanic.cpp
mechanic_LDADD = ../OpenFDM/libOpenFDM.la

quattest_SOURCES = quattest.cpp
quattest_LDADD = ../OpenFDM/libOpenFDM.la

simulationtime_SOURCES = simulationtime.cpp
simulationtime_LDADD = ../OpenFDM/libOpenFDM.la

structure_SOURCES = structure.cpp
structure_LDADD = ../OpenFDM/libOpenFDM.la

tiretestrig_SOURCES = tiretestrig.cpp
tiretestrig_LDADD = ../OpenFDM/libOpenFDM.la

unitstest_SOURCES = unitstest.cpp
unitstest_LDADD = ../OpenFDM/libOpenFDM.la
