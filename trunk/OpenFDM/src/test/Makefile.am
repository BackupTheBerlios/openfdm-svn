INCLUDES = -I$(srcdir)/..

# define some unit tests
check_PROGRAMS = quattest unitstest
TESTS          = $(check_PROGRAMS)

unitstest_SOURCES = unitstest.cpp
unitstest_LDADD = ../OpenFDM/libOpenFDM.la

quattest_SOURCES = quattest.cpp
quattest_LDADD = ../OpenFDM/libOpenFDM.la

