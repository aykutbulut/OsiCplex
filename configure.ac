# Copyright (C) 2015 Lehigh University
# All Rights Reserved.
# This file is distributed under the Eclipse Public License.

# Author:  Aykut Bulut, Lehigh University 2015-05-22

#############################################################################
#                       Names and other basic things                        #
#############################################################################

AC_PREREQ(2.59)

AC_INIT([OsiCplex],[0.0],[aykutblt@gmail.com])

AC_COPYRIGHT([/*===========================================================================*/
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

AC_REVISION(0.9)

# List one file in the package so that the configure script can test
# whether the package is actually there
AC_CONFIG_SRCDIR(configure.ac)

# Where should everything be installed by default?  Here, we want it
# to be installed directly in 'bin', 'lib', 'include' subdirectories
# of the directory where configure is run.  The default would be
# /usr/local.
AC_PREFIX_DEFAULT([`pwd`])

#############################################################################
#              Create the libtool script for the subdirectories             #
#############################################################################

AC_COIN_CREATE_LIBTOOL

#############################################################################
#                  Check which subprojects are there                        #
#############################################################################

dnl AC_COIN_MAIN_PACKAGEDIR(Blas,   ThirdParty,[daxpy.f])
dnl AC_COIN_MAIN_PACKAGEDIR(Lapack, ThirdParty,[LAPACK/SRC/dlarf.f])
dnl AC_COIN_MAIN_PACKAGEDIR(Metis,  ThirdParty,[metis-4.0/Lib/balance.c])
dnl AC_COIN_MAIN_PACKAGEDIR(Mumps,  ThirdParty,[MUMPS/src/dmumps_part1.F])
dnl AC_COIN_MAIN_PACKAGEDIR(Glpk,   ThirdParty,[glpk/src/glplpx01.c])
dnl AC_COIN_MAIN_PACKAGEDIR(Sample, Data)
dnl AC_COIN_MAIN_PACKAGEDIR(Netlib, Data)
dnl AC_COIN_MAIN_PACKAGEDIR(CoinUtils)
dnl AC_COIN_MAIN_PACKAGEDIR(Osi)
dnl AC_COIN_MAIN_PACKAGEDIR(Clp)
AC_COIN_MAIN_PACKAGEDIR(CoinUtils)
AC_COIN_MAIN_PACKAGEDIR(Sample, Data)
AC_COIN_MAIN_PACKAGEDIR(Osi)
AC_COIN_MAIN_PACKAGEDIR(Blas,   ThirdParty,[daxpy.f])
AC_COIN_MAIN_PACKAGEDIR(Lapack, ThirdParty,[LAPACK/SRC/dlarf.f])
AC_COIN_MAIN_PACKAGEDIR(Glpk,   ThirdParty,[glpk/src/glplpx01.c])

#############################################################################
#                  Check for doxygen                                        #
#############################################################################

AC_COIN_DOXYGEN(CoinUtils Osi)

##############################################################################
#                   Finishing up by writing all the output                   #
##############################################################################

# Here list all the files that configure should create (except for the
# configuration header file)
AC_CONFIG_FILES([Makefile\
	src/Makefile])
AC_CONFIG_FILES([doxydoc/doxygen.conf])

# Finally, we let configure write all the output...
AC_COIN_FINALIZE
