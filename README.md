# OsiCplex

OsiCplex is a conic solver interface for [Cplex][1] solver. OsiCplex
implements [OsiConic][2] interface, which extends Open Solver Interface (OSI)
to second order conic optimization problems.

OsiCplex depends on [CoinUtils][3], [OSI][4] and OsiConic. OsiCplex
extends COIN-OR's linear Cplex interface OsiCpx.

OsiCplex is used by [DisCO][6] to solve mixed integer conic optimization
problems.

[1]: https://www-01.ibm.com/software/commerce/optimization/cplex-optimizer/
[2]: https://github.com/aykutbulut/OSI-CONIC
[3]: https://projects.coin-or.org/CoinUtils
[4]: https://projects.coin-or.org/Osi
[6]: https://github.com/aykutbulut/DisCO

# Basic Usage

Following code snippet reads problem from an MPS file (an extended MPS
format for second order cone problems) and solves it using Cplex.

```C++
#include <OsiCplexSolverInterface.hpp>

int main(int argc, char ** argv) {
  OsiConicSolverInterface * si = new OsiCplexSolverInterface();
  si->readMps(argv[1]);
  si->initialSolve();
  std::cout << "Optimal solution is " << si->getObjValue() << std::endl;
  delete si;
  return 0;
}
```

Currently only Mosek style MPS files are supported. You can find the complete
example in the examples directory.

# Install

## Basic installation

OsiCplex is tested/works in Linux environment only for now. To install
OsiCplex you need to have a Cplex solver with a valid license installed
in your computer. You should compile OSI with Cplex first. You can do this
with the following command.

```shell
./configure --prefix=build_dir --with-cplex-incdir=/cplex_include_dir --with-cplex-lib="-L/cplex_lib_dir -lcplex -lm -lpthread"
make install
```

First command configures OSI with Cplex. You need to replace ```build_dir```
for the directory that you want to install OSI. You need to replace
```cplex_include_dir``` and ```cplex_lib_dir``` with your Cplex
directories. Second command install OSI to ```build_dir```. Please see [Osi
documentation][4] for details.

You need to configure CoinUtils and OsiConic the same way using the same ```build_dir```.

Once Osi and other dependencies are installed you can install OsiCplex with the
following command

```shell
./configure --prefix=build_dir && make install
```

OsiCplex configure script will find dependencies as long as you use same
```build_dir``` that you used during configuration of them.


## Installation Instructions for Advanced Users

If you already have the dependencies somewhere else in your computer and you do
not want to install them again, you are covered. First you need to make sure
that dependencies can be found with package config (```pkgconfig```
command). For this you need to add the directory that has ```.pc``` files of
dependencies to your ```PKG_CONFIG_PATH```. You can test whether the
dependencies are accesible with pkg-config with the following command,
```pkg-config --cflags --libs osi-cplex```.

Once the dependencies are accessible through pkg-config you can install
OsiCplex by using regular ```configure```, ```make``` and ```make install```
sequence. Configure script will find the dependencies through pkg-config and
link OsiCplex to them.

[1]: https://www-01.ibm.com/software/commerce/optimization/cplex-optimizer/
[2]: https://github.com/aykutbulut/OSI-CONIC
[3]: https://projects.coin-or.org/CoinUtils
[4]: https://projects.coin-or.org/Osi
[6]: https://github.com/aykutbulut/DisCO
