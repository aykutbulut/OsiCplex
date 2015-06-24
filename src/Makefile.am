lib_LTLIBRARIES = libOsiCplex.la

libOsiCplex_la_SOURCES=\
	OsiCplexSolverInterface.hpp\
	OsiCplexSolverInterface.cpp

libOsiCplex_la_LIBADD=\
	$(OSICONIC_LIBS)\
	$(OSICPLEX_LIBS)\
	$(OSI_LIBS)\
	$(COINUTILS_LIBS)\
	$(LIBS)

AM_CPPFLAGS=\
	-I../../include\
	$(OSICONIC_CFLAGS)\
	$(OSICPLEX_CFLAGS)\
	$(OSI_CFLAGS)\
	$(COINUTILS_CFLAGS)

# header files to be installed to $(includedir)
include_HEADERS=\
	OsiCplexSolverInterface.hpp

# this are in the disctribution but not in include
#noinst_HEADERS=
