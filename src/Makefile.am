# Copyright (C) 2015 Lehigh University.
# All Rights Reserved.
# This file is distributed under the Eclipse Public License.

# Author:  Aykut Bulut, Lehigh University 2015-05-22

AUTOMAKE_OPTIONS = foreign

########################################################################
#                               libOsiCplex                            #
########################################################################

# Name of the library compiled in this directory.  We want it to be installed
# in the 'lib' directory
lib_LTLIBRARIES = libOsiCplex.la

# List all source files for this library, including headers
libOsiCplex_la_SOURCES=\
	OsiCplexSolverInterface.hpp\
	OsiCplexSolverInterface.cpp

# List all additionally required libraries
if DEPENDENCY_LINKING
libOsiCplex_la_LIBADD = $(OSICPLEXLIB_LIBS)
endif

# This is for libtool
libOsiCplex_la_LDFLAGS = $(LT_LDFLAGS)

# Here list all include flags.
AM_CPPFLAGS = $(COINUTILS_CFLAGS)\
	 $(OSI_CFLAGS)\
	 $(OSICPX_CFLAGS)\
	 $(OSICONIC_CFLAGS)

########################################################################
#                            Additional flags                          #
########################################################################

# This line is necessary to allow VPATH compilation
DEFAULT_INCLUDES = -I. -I`$(CYGPATH_W) $(srcdir)`

########################################################################
#                Headers that need to be installed                     #
########################################################################

# Here list all the header files that are required by a user of the library,
# and that therefore should be installed in 'include/coin'
includecoindir = $(includedir)/coin
includecoin_HEADERS = \
	OsiCplexSolverInterface.hpp

install-exec-local:
	$(install_sh_DATA) config_osicplex.h $(DESTDIR)$(includecoindir)/OsiCplexConfig.h

uninstall-local:
	rm -f $(DESTDIR)$(includecoindir)/OsiCplexConfig.h
