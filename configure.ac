# Copyright (C) 2015 Lehigh University
# All Rights Reserved.
# This file is distributed under the Eclipse Public License.

# Author:  Aykut Bulut, Lehigh University 2015-05-22

#############################################################################
#                       Names and other basic things                        #
#############################################################################
AC_PREREQ(2.59)
AC_INIT([OsiCplex],[0.0],[aykutblt@gmail.com])

AC_COPYRIGHT([
/*===========================================================================*/
/*                                                                           */
/* OsiCplex is developed by Aykut Bulut (aykutblt@gmail.com)                 */
/*                                                                           */
/* (c) Copyright 2015 Aykut Bulut. All Rights Reserved.                      */
/*                                                                           */
/* This software is licensed under the Eclipse Public License. Please see    */
/* accompanying file for terms.                                              */
/*                                                                           */
/*===========================================================================*/
])

# List one file in the package so that the configure script can test
# whether the package is actually there
AC_CONFIG_SRCDIR(src/OsiCplexSolverInterface.hpp)

# Where should everything be installed by default?  Here, we want it
# to be installed directly in 'bin', 'lib', 'include' subdirectories
# of the directory where configure is run.  The default would be
# /usr/local.
AC_PREFIX_DEFAULT([`pwd`])

#############################################################################
#                         Standard build tool stuff                         #
#############################################################################
# Get the system type
AC_CANONICAL_BUILD

# If this project depends on external projects, the Externals file in
# the source root directory contains definition of where to find those
# externals.  The following macro ensures that those externals are
# retrieved by svn if they are not there yet.
AC_COIN_PROJECTDIR_INIT(OsiCplex)

# Check if user wants to produce debugging code
AC_COIN_DEBUG_COMPILE(OsiCplex)

# Get the name of the C++ compiler and appropriate compiler options
AC_COIN_PROG_CXX

# Initialize automake and libtool
AC_COIN_INIT_AUTO_TOOLS

#############################################################################
#                              COIN-OR components                           #
#############################################################################
AC_COIN_CHECK_PACKAGE(CoinUtils, [coinutils], [OsiCplexLib])
if test $coin_has_coinutils != yes ; then
  AC_MSG_ERROR([Required package CoinUtils not available.])
fi
AC_COIN_CHECK_PACKAGE(Osi,       [osi],       [OsiCplexLib])
if test $coin_has_osi != yes ; then
  AC_MSG_ERROR([Required package Osi not available.])
fi
AC_COIN_CHECK_PACKAGE(OsiCpx,    [osi-cplex],  [OsiCplexLib])
if test $coin_has_osi != yes ; then
  AC_MSG_ERROR([Required package OsiCpx not available.])
fi
AC_COIN_CHECK_PACKAGE(OsiConic,  [osiconic],  [OsiCplexLib])
if test $coin_has_osiconic != yes ; then
  AC_MSG_ERROR([Required package OsiConic not available.])
fi

##############################################################################
#                   VPATH links for example input files                      #
##############################################################################
# In case this is a VPATH configuration we need to make sure that the
# input files for the examples are available in the VPATH directory.
## AC_COIN_VPATH_LINK(examples/input.mps)

#############################################################################
#                  Check for doxygen                                        #
#############################################################################
AC_COIN_DOXYGEN(CoinUtils Osi OsiCpx OsiConic)

##############################################################################
#                   Finishing up by writing all the output                   #
##############################################################################
# Here list all the files that configure should create (except for the
# configuration header file)
AC_CONFIG_FILES([Makefile\
	examples/Makefile\
	src/Makefile\
	osicplex.pc
	osicplex-uninstalled.pc])

AC_CONFIG_FILES([doxydoc/doxygen.conf])

# Here put the location and name of the configuration header file
AC_CONFIG_HEADER([src/config.h src/config_osicplex.h])

# Finally, we let configure write all the output...
AC_COIN_FINALIZE
