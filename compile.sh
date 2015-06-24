#!/bin/bash
mkdir build
cd build
cplex_inc_dir=/usr/local/cplex/include/ilcplex
cplex_lib_dir=-L/usr/local/cplex/lib/x86-64_linux/static_pic/ -lcplex -lm -lpthread
build_dir=$PWD
inc_dir=${build_dir%%/}/include
lib_dir=${build_dir%%/}/lib
pkg_dir=${lib_dir%%/}/pkgconfig
PKG_CONFIG_PATH=${pkg_dir}:$PKG_CONFIG_PATH
export CXXFLAGS="-std=c++11 -g"
# configure and install CoinUtils
mkdir CoinUtils
cd CoinUtils
../../CoinUtils/configure --prefix=$build_dir
make -j 10 install
cd ..
# configure and install OsiMsk
mkdir Osi
cd Osi
../../Osi/configure --with-cplex-incdir=/usr/local/cplex/include/ilcplex --with-cplex-lib="-L/usr/local/cplex/lib/x86-64_linux/static_pic/ -lcplex -lm -lpthread" --prefix=$build_dir
make -j 10 install
cd ..
#configure and install OsiConic
mkdir OsiConic
cd OsiConic
../../OsiConic/configure --prefix=$build_dir
make -j 10 install
cd ..
#configure and install OsiCplex
mkdir OsiCplex
cd OsiCplex
../../configure --prefix=$build_dir
make -j 10 install
cd ..
