AC_PREREQ([2.69])
AC_INIT([osicplex], [0.1], [aykutblt@gmail.com])
AC_CONFIG_SRCDIR([src/OsiCplexSolverInterface.hpp])
AM_INIT_AUTOMAKE([foreign -Wall -Werror])
AC_CONFIG_HEADERS([config.h])

# for creating library
AC_PROG_LIBTOOL

# Checks for programs.
AC_PROG_CXX

# Installs to ./bin, changes default behavior
AC_PREFIX_DEFAULT([`pwd`])

# Checks for header files.
AC_CHECK_HEADERS([math.h])

# Checks for typedefs, str
AC_HEADER_STDBOOL
AC_C_INLINE
AC_TYPE_SIZE_T

# Check dependent libraries through pkg-config
PKG_CHECK_MODULES([OSICONIC], [osiconic >= 0.0])
PKG_CHECK_MODULES([OSICPLEX], [osi-cplex >= 0.105])
PKG_CHECK_MODULES([OSI], [osi >= 0.105])
PKG_CHECK_MODULES([COINUTILS], [coinutils >= 2.8])

AC_CONFIG_FILES([Makefile
	examples/Makefile
	src/Makefile
	osicplex.pc
	osicplex-uninstalled.pc])

AC_OUTPUT
